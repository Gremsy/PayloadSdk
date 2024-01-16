#include <iostream>
#include <string>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <regex>
#include <string>
#include <vector>

CURL *curl;
FILE *fp;
CURLcode res;
std::vector<std::string> hrefElements;
bool listed = false;
std::string downloadDirectory;

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

void directory_listing(std::string url) 
{
    hrefElements.clear();
    listed = true;
    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    // Set the callback function to write the downloaded data to the file
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // Perform the request
    res = curl_easy_perform(curl);

    // Check for errors
    if (res != CURLE_OK)
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
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
}

void dowload_file(std::string url, std::string fileName) 
{
    char outfilename[FILENAME_MAX];
    // Set the URL
    url = url + "/download/" + fileName;
    strcpy(outfilename, fileName.c_str());

    const size_t fileNameLength = strlen(outfilename);
    const size_t bufferSize = fileNameLength + 1;
    char replace20toSpace[bufferSize];
    size_t replacedIndex = 0;
    // Replace "%20" with a space
    for (size_t i = 0; i < fileNameLength; i++) {
        if (fileName[i] == '%' && fileName[i + 1] == '2' && fileName[i + 2] == '0') {
            replace20toSpace[replacedIndex++] = ' '; // Replace "%20" with a space
            i += 2; // Skip the next two characters: '2' and '0'
        } else {
            replace20toSpace[replacedIndex++] = fileName[i];
        }
    }
    replace20toSpace[replacedIndex] = '\0'; // Null-terminate the replace20toSpace string
    if (!downloadDirectory.empty()) {
        char lastChar = downloadDirectory.back();
        if (lastChar != '/') {
            downloadDirectory = downloadDirectory + "/";
        }
        // Concatenate the strings
        std::string concatenated = downloadDirectory + replace20toSpace;
        // Copy the concatenated string back to ch1
        std::strcpy(replace20toSpace, concatenated.c_str());
    }
    std::cout << replace20toSpace << std::endl;
    fp = fopen(replace20toSpace,"wb");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    // Set the callback function to write the downloaded data to the file
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt (curl, CURLOPT_VERBOSE, 1L);

    // Perform the request
    res = curl_easy_perform(curl);

    // Check for errors
    if (res != CURLE_OK) 
        std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;

    fclose(fp);
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

int main(int argc, char* argv[]) {
    std::string ip;
    if (argc == 1) {
        std::cout << "The download directory is in build folder." << std::endl;
        std::cout << "Input Payload IP (xxx.xxx.xxx.xxx): ";
        std::cin >> ip;
    }
    else if (argc == 2) {
        if (isDirectoryPath(argv[1])) {
            downloadDirectory = argv[1];
            std::cout << "The download directory: " << downloadDirectory << std::endl;
            std::cout << "Input Payload IP (xxx.xxx.xxx.xxx): ";
            std::cin >> ip;
        } else {
            if (isIPAddress(argv[1])) {
                std::cout << "The download directory is in build folder." << std::endl;
                ip = argv[1];
            }
        }
    } else {
        if (isDirectoryPath(argv[1])) {
            downloadDirectory = argv[1];
            std::cout << "The download directory: " << downloadDirectory << std::endl;
            ip = argv[2];
        } else {
            ip = argv[1];
            downloadDirectory = argv[2];
            std::cout << "The download directory: " << downloadDirectory << std::endl;
        }
    }
    std::cout << "IP Address: " << ip << std::endl;
    std::string url = "http://" + ip + ":8000";
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    std::string choice;
    std::string name;
    if (curl) {
        while (true) {
            std::cout << "" << std::endl;
            std::cout << "----" << std::endl;
            std::cout << "Select an option:" << std::endl;
            std::cout << "  1. List meida files" << std::endl;
            std::cout << "  2. Download a Image or a Video " << std::endl;
            std::cout << "  3. Download all Images" << std::endl;
            std::cout << "  4. Download all Videos" << std::endl;
            std::cout << "  Enter 'q' to quit" << std::endl;
            std::cout << "Choice: ";
            std::cin >> choice;
            std::cout << "" << std::endl;
            if (choice == "1") {
                std::cout << "Listing items..." << std::endl;
                directory_listing(url+"/list-file");
                std::cout << "" << std::endl;
                for (const auto& element : hrefElements) {
                    std::cout << element << std::endl;
                }

            } else if (choice == "2") {
                directory_listing(url+"/list-file");
                std::cout << "" << std::endl;
                for (const auto& element : hrefElements) {
                    std::cout << element << std::endl;
                }
                std::cout << "--" << std::endl;
                std::cout << "Downloading a image or video. Enter the name: ";
                std::cin >> name;
                dowload_file(url, name);

            } else if (choice == "3") {
                // Process "Download Video" option
                std::cout << "Downloading all Images..." << std::endl;
                if (listed) {
                    for (const auto& element : hrefElements) {
                        if (isImageExtension(element))
                            dowload_file(url, element);
                    }
                } else {
                    directory_listing(url+"/list-file");
                    std::cout << "" << std::endl;
                    for (const auto& element : hrefElements) {
                        if (isImageExtension(element))
                            dowload_file(url, element);
                    }
                }

            } else if (choice == "4") {
                // Process "Download Video" option
                std::cout << "Downloading all Videos..." << std::endl;
                if (listed) {
                    for (const auto& element : hrefElements) {
                        if (isVideoExtension(element))
                            dowload_file(url, element);
                    }
                } else {
                    directory_listing(url+"/list-file");
                    std::cout << "" << std::endl;
                    for (const auto& element : hrefElements) {
                        if (isVideoExtension(element))
                            dowload_file(url, element);
                    }
                }

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