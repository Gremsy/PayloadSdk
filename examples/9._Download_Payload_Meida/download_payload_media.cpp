#include <iostream>
#include <string>
#include <curl/curl.h>
#include <curl/easy.h>
#include <jsoncpp/json/json.h>
#include <cstring>  // For strcpy()

CURL *curl;
FILE *fp;
CURLcode res;

std::string GetStringAfterSlash(const std::string& input) {
    size_t lastSlashPos = input.find_last_of('/');
    if (lastSlashPos != std::string::npos && lastSlashPos < input.length() - 1) {
        return input.substr(lastSlashPos + 1);
    }
    return "";
}
// Callback function to receive the response
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    response->append(static_cast<char*>(contents), totalSize);
    return totalSize;
}
// Callback function to download the data
size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
    size_t written;
    written = fwrite(ptr, size, nmemb, stream);
    return written;
}

int main(int argc, char* argv[]) {
    std::string event = argv[1];
    if (event=="list") {
        std::string ip = argv[2];
        std::string url = "http://" + ip + "/list_files";  // Replace with the actual URL

        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();

        if (curl) {
            std::string response;
            // Set the URL
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            // Set the POST data
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");
            // Set the callback function to write the downloaded data to the file
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            // Perform the request
            res = curl_easy_perform(curl);

            // Check for errors
            if (res != CURLE_OK)
                std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
            else 
                // Process the response
                std::cout << "Listing /sdcard: " << response << std::endl;
            // Cleanup
            curl_easy_cleanup(curl);
        }
    }
    else if (event=="download") {
        std::string ip = argv[2];
        std::string tree = argv[3];
        char outfilename[FILENAME_MAX];
        strcpy(outfilename, tree.c_str());
        if (tree[0]=='/') {
            std::string result = GetStringAfterSlash(tree);
            strcpy(outfilename, result.c_str());
        }

        Json::Value postData;
        std::string data = outfilename;
        postData["file_path"] = data;
        // Serialize the dictionary to JSON
        Json::StreamWriterBuilder writer;
        std::string postDataJson = Json::writeString(writer, postData);

        if (argv[4]!=NULL) {
            std::string directory = argv[4];
            char lastChar = directory.back();
            if (lastChar!='/') 
                directory = directory + "/" + outfilename;
            else 
                directory = directory + outfilename;
            strcpy(outfilename, directory.c_str());

        }
        std::string url = "http://" + ip + "/download_file";

        curl = curl_easy_init();
        if (curl)
        {
            fp = fopen(outfilename,"wb");
            // Set the URL
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            // Set the POST data
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postDataJson.c_str());
            // Set the callback function to write the downloaded data to the file
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
            curl_easy_setopt (curl, CURLOPT_VERBOSE, 1L);
            // Perform the request
            res = curl_easy_perform(curl);
            // Check for errors
            if (res != CURLE_OK) 
                std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
            else 
                // Process the response
                std::cout << "Response: " << fp << std::endl;
            // Cleanup
            curl_easy_cleanup(curl);
            fclose(fp);
        }
    }
    curl_global_cleanup();
    return 0;
}