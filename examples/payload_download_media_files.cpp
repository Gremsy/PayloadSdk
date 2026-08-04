#include <iostream>
#include <string>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <regex>
#include <string>
#include <cstring>
#include <vector>
#include <limits>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include "payloadsdk.h"

// Some payloads ask for a login before serving media files. Probe the server at
// run time instead of relying on a build flag. Credentials are typed in by the
// user and never written to disk.
#define LOGIN_MAX_ATTEMPTS 3

CURL *curl;
FILE *fp;
CURLcode res;
std::vector<std::string> hrefElements;
bool listed = false;
std::string downloadDirectory;
// Base URL "http://<ip>:8000", needed to sign in again when a session expires.
std::string baseUrl;

// Callback function to receive the response
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    response->append(static_cast<char*>(contents), totalSize);
    return totalSize;
}
// Callback function to download the data
size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    size_t written = fwrite(ptr, size, nmemb, stream);
    return written;
}

// Reset the per-request options that the previous call may have left behind,
// so a download never inherits the settings of a listing request (or vice versa).
void reset_request_options() {
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, NULL);
    // CURLOPT_POSTFIELDS turns the handle back into a POST, so clear it first and
    // keep CURLOPT_HTTPGET last - otherwise every request after a login is a POST.
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, NULL);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, 0L);
    curl_easy_setopt(curl, CURLOPT_POST, 0L);
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 0L);
    // A redirect means the session is gone - report it instead of saving the
    // login page under the media file's name.
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
}

// An unauthenticated page comes back with HTTP 200, so the status code alone
// cannot tell a valid session from an expired one.
bool is_login_page(const std::string& html) {
    return html.find("/smb-login") != std::string::npos ||
           html.find("smb-username") != std::string::npos;
}

// Read a line without echoing it back to the terminal.
std::string read_hidden_line() {
    termios oldTerm;
    bool restoreTerm = (tcgetattr(STDIN_FILENO, &oldTerm) == 0);
    if (restoreTerm) {
        termios newTerm = oldTerm;
        newTerm.c_lflag &= ~ECHO;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &newTerm);
    }

    std::string line;
    std::getline(std::cin, line);

    if (restoreTerm) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &oldTerm);
    }
    std::cout << std::endl; // the user's Enter was not echoed
    return line;
}

// Post the credentials. On success the server sets a session cookie that curl's
// cookie engine replays on the following requests.
bool smb_login(const std::string& baseUrl, const std::string& user,
               const std::string& password, std::string& errorOut) {
    Json::Value payload;
    payload["username"] = user;
    payload["password"] = password;

    Json::StreamWriterBuilder writerBuilder;
    writerBuilder["indentation"] = "";
    const std::string body = Json::writeString(writerBuilder, payload);

    std::string response;
    curl_slist* headers = curl_slist_append(NULL, "Content-Type: application/json");

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, (baseUrl + "/smb-login").c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

    res = curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, NULL);
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);

    if (res != CURLE_OK) {
        errorOut = curl_easy_strerror(res);
        return false;
    }

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string parseErrors;
    std::unique_ptr<Json::CharReader> reader(readerBuilder.newCharReader());
    const bool parsed = reader->parse(response.c_str(),
                                      response.c_str() + response.size(),
                                      &root, &parseErrors);

    if (parsed && root.get("ok", false).asBool()) {
        return true;
    }

    if (parsed && root.isMember("error")) {
        errorOut = root["error"].asString();
    } else {
        errorOut = "unexpected reply from server (HTTP " + std::to_string(httpCode) + ")";
    }
    return false;
}

// Ask for the credentials and retry a few times on rejection.
bool do_interactive_login(const std::string& baseUrl) {
    std::cout << "This payload requires a login to access media files." << std::endl;

    for (int attempt = 1; attempt <= LOGIN_MAX_ATTEMPTS; attempt++) {
        std::string user;
        std::string password;

        std::cout << "Username: ";
        std::getline(std::cin, user);
        std::cout << "Password: ";
        password = read_hidden_line();

        if (user.empty() || password.empty()) {
            std::cout << "Username and password must not be empty." << std::endl;
            continue;
        }

        std::string error;
        if (smb_login(baseUrl, user, password, error)) {
            std::cout << "Login successful." << std::endl;
            return true;
        }

        std::cerr << "Login failed: " << error
                  << " (attempt " << attempt << "/" << LOGIN_MAX_ATTEMPTS << ")" << std::endl;
    }
    return false;
}

// Sign in only when the server asks for it. An unauthenticated page answers 200
// with the login form; a payload still detecting its cameras answers 404.
bool ensure_authenticated(const std::string& url) {
    std::string response;

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, (url + "/list-file").c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Cannot reach the media server: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    if (httpCode == 404) {
        std::cerr << "Media files are not available on this payload: the payload type "
                     "has not been detected yet. Wait for the payload to finish starting up "
                     "and try again." << std::endl;
        return false;
    }
    if (httpCode != 200) {
        std::cerr << "Media server replied HTTP " << httpCode << std::endl;
        return false;
    }
    if (!is_login_page(response)) {
        return true;    // this payload serves the media files without a login
    }
    return do_interactive_login(url);
}

bool directory_listing(std::string url, bool allowRetry = true)
{
    hrefElements.clear();
    listed = false;
    std::string response;

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    // Set the callback function to write the downloaded data to the file
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

    // Perform the request
    res = curl_easy_perform(curl);

    // Check for errors
    if (res != CURLE_OK) {
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
    if (httpCode != 200) {
        std::cerr << "Listing failed: HTTP " << httpCode << std::endl;
        return false;
    }

    // An expired session returns the login page with a 200 status, which would
    // otherwise look like an empty storage. Sign in again and retry once.
    if (is_login_page(response)) {
        std::cerr << "The session has expired." << std::endl;
        if (!allowRetry || !do_interactive_login(baseUrl)) {
            return false;
        }
        return directory_listing(url, false);
    }

    // Regular expression pattern
    // std::regex pattern("<a href=\"(/delete/.*?)\" class=\"delete-link\" onclick=\"return confirm('Are you sure you want to delete this image?')\">[Delete]</a>");
    std::regex pattern("<a href=\"(/delete/.*?)\" class=\"delete-link\"");
    std::smatch match;
    std::string::const_iterator searchStart(response.cbegin());
    while (std::regex_search(searchStart, response.cend(), match, pattern)) {
        if (match.size() > 1) {
            std::string imageURL = match[1];
            std::string imageName = imageURL.substr(imageURL.find_last_of('/') + 1);
            // Replace space with a "%20", in html space symbol is "%20"
            std::string replaceSpaceto20;
            for (char c : imageName) {
                if (c == ' ') {
                    replaceSpaceto20 += "%20";
                } else {
                    replaceSpaceto20 += c;
                }
            }
            imageName = replaceSpaceto20;
            hrefElements.push_back(imageName);
        }
        searchStart = match.suffix().first;
    }

    listed = true;
    return true;
}

bool dowload_file(std::string url, std::string fileName, bool allowRetry = true)
{
    const std::string requestUrl = url + "/download/" + fileName;

    // Replace "%20" with a space to build the local file name
    std::string localName;
    for (size_t i = 0; i < fileName.size(); i++) {
        if (fileName.compare(i, 3, "%20") == 0) {
            localName += ' ';
            i += 2; // Skip the next two characters: '2' and '0'
        } else {
            localName += fileName[i];
        }
    }

    std::string outPath = localName;
    if (!downloadDirectory.empty()) {
        if (downloadDirectory.back() != '/') {
            downloadDirectory += "/";
        }
        outPath = downloadDirectory + localName;
    }
    std::cout << outPath << std::endl;

    fp = fopen(outPath.c_str(), "wb");
    if (fp == NULL) {
        std::cerr << "Cannot open '" << outPath << "' for writing: "
                  << strerror(errno) << std::endl;
        return false;
    }

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, requestUrl.c_str());

    // Set the callback function to write the downloaded data to the file
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    // Videos can be large, so bound the transfer by stall time instead of a
    // fixed deadline.
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 30L);

    // Perform the request
    res = curl_easy_perform(curl);

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    char* contentType = NULL;
    curl_easy_getinfo(curl, CURLINFO_CONTENT_TYPE, &contentType);
    const bool gotHtml = (contentType != NULL && strstr(contentType, "text/html") != NULL);

    fclose(fp);
    fp = NULL;

    // Check for errors
    if (res != CURLE_OK) {
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
        remove(outPath.c_str());
        return false;
    }

    // A redirect here means the session expired mid-session; without this check
    // the login page would be saved under the media file's name.
    if (httpCode == 301 || httpCode == 302 || httpCode == 303 || httpCode == 307) {
        std::cerr << "Download rejected: session expired, please restart and log in again."
                  << std::endl;
        remove(outPath.c_str());
        return false;
    }

    // Without this the login page would be saved under the media file's name and
    // look like a corrupt photo. Sign in again and retry once.
    if (gotHtml) {
        remove(outPath.c_str());
        std::cerr << "The session has expired." << std::endl;
        if (!allowRetry || !do_interactive_login(baseUrl)) {
            return false;
        }
        return dowload_file(url, fileName, false);
    }

    if (httpCode != 200) {
        std::cerr << "Download failed: HTTP " << httpCode << std::endl;
        remove(outPath.c_str());
        return false;
    }

    return true;
}

// Read the storage choices straight out of the media page, so the SDK does not
// need to know which payload models have which storage.
bool parse_storage_options(const std::string& html,
                           std::vector<std::pair<std::string, std::string> >& options,
                           std::string& current) {
    options.clear();
    current.clear();

    const size_t selectPos = html.find("id=\"storage-select\"");
    if (selectPos == std::string::npos) {
        return false;                       // this payload has a single storage
    }
    const size_t endPos = html.find("</select>", selectPos);
    if (endPos == std::string::npos) {
        return false;
    }
    const std::string block = html.substr(selectPos, endPos - selectPos);

    std::regex pattern("<option value=\"([^\"]*)\"([^>]*)>([^<]*)</option>");
    std::smatch match;
    std::string::const_iterator searchStart(block.cbegin());
    while (std::regex_search(searchStart, block.cend(), match, pattern)) {
        const std::string value = match[1];
        const std::string attrs = match[2];
        const std::string label = match[3];
        options.push_back(std::make_pair(value, label));
        if (attrs.find("selected") != std::string::npos) {
            current = value;
        }
        searchStart = match.suffix().first;
    }
    return options.size() > 1;              // nothing to choose from otherwise
}

// Tell the server which storage the following listing and download requests apply to.
bool set_storage_source(const std::string& url, const std::string& option,
                        bool allowRetry = true) {
    Json::Value payload;
    payload["option"] = option;

    Json::StreamWriterBuilder writerBuilder;
    writerBuilder["indentation"] = "";
    const std::string body = Json::writeString(writerBuilder, payload);

    std::string response;
    curl_slist* headers = curl_slist_append(NULL, "Content-Type: application/json");

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, (url + "/process-option").c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

    res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, NULL);

    if (res != CURLE_OK) {
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    if (is_login_page(response)) {
        std::cerr << "The session has expired." << std::endl;
        if (!allowRetry || !do_interactive_login(baseUrl)) {
            return false;
        }
        return set_storage_source(url, option, false);
    }

    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string parseErrors;
    std::unique_ptr<Json::CharReader> reader(readerBuilder.newCharReader());
    const bool parsed = reader->parse(response.c_str(),
                                      response.c_str() + response.size(),
                                      &root, &parseErrors);
    if (parsed && root.get("ok", false).asBool()) {
        listed = false;                     // the cached listing belongs to the old storage
        std::cout << "Storage source: " << root.get("media_source", option).asString()
                  << " (" << root.get("directory", "").asString() << ")" << std::endl;
        return true;
    }

    std::cerr << "Could not switch storage: "
              << (parsed && root.isMember("error") ? root["error"].asString() : response)
              << std::endl;
    return false;
}

// Ask which storage to work on. Does nothing when the payload has only one.
void choose_storage_source(const std::string& url) {
    std::string html;

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, (url + "/list-file").c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &html);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    if (curl_easy_perform(curl) != CURLE_OK) {
        return;
    }

    std::vector<std::pair<std::string, std::string> > options;
    std::string current;
    if (!parse_storage_options(html, options, current)) {
        return;
    }

    std::cout << "" << std::endl;
    std::cout << "This payload has more than one storage. Select the one to work on:" << std::endl;
    for (size_t i = 0; i < options.size(); i++) {
        std::cout << "  " << (i + 1) << ". " << options[i].second;
        if (options[i].first == current) {
            std::cout << "  (current)";
        }
        std::cout << std::endl;
    }
    std::cout << "Choice (Enter to keep the current one): ";

    std::string answer;
    std::getline(std::cin, answer);
    if (answer.empty()) {
        return;
    }

    const long picked = strtol(answer.c_str(), NULL, 10);
    if (picked < 1 || picked > (long)options.size()) {
        std::cout << "Invalid choice, keeping the current storage." << std::endl;
        return;
    }
    if (options[picked - 1].first == current) {
        return;
    }
    set_storage_source(url, options[picked - 1].first);
}

// Deleting is irreversible, so make the user type the whole word.
bool confirm_destructive(const std::string& what) {
    std::cout << what << std::endl
              << "This cannot be undone. Type 'yes' to confirm: ";
    std::string answer;
    std::cin >> answer;
    if (answer != "yes") {
        std::cout << "Cancelled." << std::endl;
        return false;
    }
    return true;
}

// Delete request: /delete/<name> is a GET, the "delete all" endpoints are POSTs.
// Success is a redirect back to the listing.
bool send_delete(const std::string& requestUrl, bool usePost, bool allowRetry = true) {
    std::string response;

    reset_request_options();
    curl_easy_setopt(curl, CURLOPT_URL, requestUrl.c_str());
    if (usePost) {
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, 0L);
    }
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    if (httpCode == 200 && is_login_page(response)) {
        std::cerr << "The session has expired." << std::endl;
        if (!allowRetry || !do_interactive_login(baseUrl)) {
            return false;
        }
        return send_delete(requestUrl, usePost, false);
    }

    // 302 = redirected back to the listing, 200 = plain acknowledgement
    if (httpCode == 200 || httpCode == 302) {
        listed = false;     // the cached listing is stale now
        return true;
    }

    std::cerr << "Delete failed: HTTP " << httpCode << std::endl;
    return false;
}

bool isImageExtension(const std::string& fileName) {
    // List of common image extensions
    std::vector<std::string> imageExtensions = {".jpg", ".jpeg", ".png", ".bmp", ".gif"};

    // Extract the file extension from the file name
    size_t dotIndex = fileName.find_last_of(".");
    if (dotIndex != std::string::npos) {
        std::string extension = fileName.substr(dotIndex);

        // Check if the extension is in the list of image extensions
        for (const std::string& imageExtension : imageExtensions) {
            if (extension == imageExtension) {
                return true;
            }
        }
    }
    return false;
}

bool isVideoExtension(const std::string& fileName) {
    // List of common image extensions
    std::vector<std::string> videoExtensions = {".mp4", ".avi", ".mov", ".mkv", ".wmv"};

    // Extract the file extension from the file name
    size_t dotIndex = fileName.find_last_of(".");
    if (dotIndex != std::string::npos) {
        std::string extension = fileName.substr(dotIndex);

        // Check if the extension is in the list of image extensions
        for (const std::string& videoExtension : videoExtensions) {
            if (extension == videoExtension) {
                return true;
            }
        }
    }

    return false;
}

bool isDirectoryPath(const std::string& str) {
    // Check if the string starts with a '/' character, which is common for directory paths
    return !str.empty() && str[0] == '/';
}

bool isIPAddress(const std::string& str) {
    // Check if the string matches the pattern of an IP address
    // A simple pattern matching can be performed using regular expressions
    // This pattern assumes IPv4 addresses
    std::regex ipPattern("^\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$");
    return std::regex_match(str, ipPattern);
}

// Download every listed file matching the given extension filter, refreshing
// the listing first when it has not been fetched yet.
void download_all(const std::string& url, bool (*matches)(const std::string&)) {
    if (!listed && !directory_listing(url + "/list-file")) {
        return;
    }
    for (const auto& element : hrefElements) {
        if (matches(element)) {
            dowload_file(url, element);
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "The download directory is in build folder." << std::endl;
    }
    else {
        if (isDirectoryPath(argv[1])) {
            downloadDirectory = argv[1];
            std::cout << "The download directory: " << downloadDirectory << std::endl;
        } else {
            std::cout << "The download directory is in build folder." << std::endl;
        }
    }
    std::cout << "IP Address: " << std::string(udp_ip_target) << std::endl;
    std::string url = "http://" + std::string(udp_ip_target) + ":8000";
    baseUrl = url;
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    std::string choice;
    std::string name;
    if (curl) {
        // In-memory cookie engine: keeps the session cookie for later requests.
        curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "");

        // Sign in only when the payload actually asks for it.
        if (!ensure_authenticated(url)) {
            std::cerr << "Could not access the media server. Exiting." << std::endl;
            curl_easy_cleanup(curl);
            curl_global_cleanup();
            return 1;
        }

        // Payloads with both internal flash and an SD card let the user pick one.
        choose_storage_source(url);

        while (true) {
            std::cout << "" << std::endl;
            std::cout << "----" << std::endl;
            std::cout << "Select an option:" << std::endl;
            std::cout << "  1. List media files" << std::endl;
            std::cout << "  2. Download a Image or a Video " << std::endl;
            std::cout << "  3. Download all Images" << std::endl;
            std::cout << "  4. Download all Videos" << std::endl;
            std::cout << "  5. Delete a Image or a Video" << std::endl;
            std::cout << "  6. Delete all Images" << std::endl;
            std::cout << "  7. Delete all Videos" << std::endl;
            std::cout << "  8. Change storage source (Internal / SD Card)" << std::endl;
            std::cout << "  Enter 'q' to quit" << std::endl;
            std::cout << "Choice: ";
            std::cin >> choice;
            std::cout << "" << std::endl;
            if (choice == "1") {
                std::cout << "Listing items..." << std::endl;
                if (directory_listing(url+"/list-file")) {
                    std::cout << "" << std::endl;
                    for (const auto& element : hrefElements) {
                        std::cout << element << std::endl;
                    }
                    if (hrefElements.empty()) {
                        std::cout << "(no media files on the payload)" << std::endl;
                    }
                }
            } else if (choice == "2") {
                if (directory_listing(url+"/list-file")) {
                    std::cout << "" << std::endl;
                    for (const auto& element : hrefElements) {
                        std::cout << element << std::endl;
                    }
                    std::cout << "--" << std::endl;
                    std::cout << "Downloading a image or video. Enter the name: ";
                    std::cin >> name;
                    dowload_file(url, name);
                }
            } else if (choice == "3") {
                // Process "Download Image" option
                std::cout << "Downloading all Images..." << std::endl;
                download_all(url, isImageExtension);
            } else if (choice == "4") {
                // Process "Download Video" option
                std::cout << "Downloading all Videos..." << std::endl;
                download_all(url, isVideoExtension);
            } else if (choice == "5") {
                if (directory_listing(url + "/list-file")) {
                    std::cout << "" << std::endl;
                    for (const auto& element : hrefElements) {
                        std::cout << element << std::endl;
                    }
                    if (hrefElements.empty()) {
                        std::cout << "(no media files on the payload)" << std::endl;
                        continue;
                    }
                    std::cout << "--" << std::endl;
                    std::cout << "Deleting a image or video. Enter the name: ";
                    std::cin >> name;
                    if (confirm_destructive("Delete '" + name + "' from the payload?") &&
                        send_delete(url + "/delete/" + name, false)) {
                        std::cout << "Deleted." << std::endl;
                    }
                }
            } else if (choice == "6") {
                if (confirm_destructive("Delete ALL images on the payload?") &&
                    send_delete(url + "/delete-all-images", true)) {
                    std::cout << "All images deleted." << std::endl;
                }
            } else if (choice == "7") {
                if (confirm_destructive("Delete ALL videos on the payload?") &&
                    send_delete(url + "/delete-all-videos", true)) {
                    std::cout << "All videos deleted." << std::endl;
                }
            } else if (choice == "8") {
                // choose_storage_source() reads a whole line, so drop the queued newline.
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                choose_storage_source(url);
                continue;   // the newline is already consumed
            } else if (choice == "q") {
                // Quit the program
                break;
            } else {
                std::cout << "Invalid choice. Please try again." << std::endl;
            }

            // Ignore any remaining characters in the input buffer
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        curl_easy_cleanup(curl);
        curl_global_cleanup();
    }
    return 0;
}
