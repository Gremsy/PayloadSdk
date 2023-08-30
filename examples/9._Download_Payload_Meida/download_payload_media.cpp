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
    std::regex pattern("<a href=\"(.*?)\">");
    // Extract elements after href using regular expression matching
    std::sregex_iterator it(response.begin(), response.end(), pattern);
    std::sregex_iterator end;

    // Add the extracted elements
    while (it != end) {
        std::smatch match = *it;
        hrefElements.push_back(match[1]);
        ++it;
    }
}

void dowload_file(std::string url, std::string fileName) 
{
    char outfilename[FILENAME_MAX];
    // Set the URL
    url = url + "/" + fileName;
    strcpy(outfilename, fileName.c_str());

    fp = fopen(outfilename,"wb");
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


int main(int argc, char* argv[]) {
    std::string url = "http://192.168.12.100:8000";
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (curl) {
        // List all files in SDCARD
        directory_listing(url);
        for (const auto& element : hrefElements) {
            std::cout << element << std::endl;
        }
        // Download first element
        dowload_file(url, hrefElements[0]);
    }
    curl_easy_cleanup(curl);
    curl_global_cleanup();
    return 0;
}