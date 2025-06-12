#include "main.h"

void* raw_data_process_thread(void *args)
{
    std::vector<uint16_t> data;
    int rows = HEIGHT;
    int cols = WIDTH;

    cursor_point.x = WIDTH/2;
    cursor_point.y = HEIGHT/2;
    while(1){
		int qsize = checkRawDataQueue(data);
        if(qsize > 0){
            cv::Mat imageRGB(HEIGHT, WIDTH, CV_8UC3);

            // Get Max Temp and Min Temp
            getTempImage(data);

            // Apply Palette Color to Raw Data
            applyPalette(data, imageRGB, colorpalette_rgba);

            drawTempPoint(imageRGB, coldest_point, cv::Scalar(0, 0, 255));
            drawTempPoint(imageRGB, hottest_point, cv::Scalar(255, 0, 0));
            drawTempPoint(imageRGB, cursor_point, cv::Scalar(0, 255, 0));

            cv::cvtColor(imageRGB, imageRGB, cv::COLOR_RGB2BGR);
            cv::imshow("Image", imageRGB);
            cv::waitKey(1);
        }
        else usleep(100000);
    }
}

int main() {
    // create raw data process thread
    pthread_create(&thrd_frame_process, NULL, &raw_data_process_thread, (int *)1);

    int clientSocket;
    sockaddr_in serverAddr;
    char buffer[BUFFER_SIZE];

    // Create socket
    if ((clientSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    // Config Vio IP Address 
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PAYLOAD_PORT);
    if (inet_pton(AF_INET, PAYLOAD_IP, &serverAddr.sin_addr) <= 0) {
        perror("Invalid address or address not supported");
        return -1;
    }

    // Connect Vio Payload
    if (connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Connection failed");
        return -1;
    }

    printf("Connected Vio Payload IP %s - Port %d\n", PAYLOAD_IP, PAYLOAD_PORT);

    while(1){
        
        // Receive data size
        size_t dataSize;
        recv(clientSocket, &dataSize, sizeof(dataSize), 0);

        // Create buffer to receive data
        std::vector<uint8_t> socketData(dataSize / sizeof(uint8_t));

        // Receive socket data 
        socketRecvAll(clientSocket, reinterpret_cast<char*>(socketData.data()), dataSize);

        // Decompress data to Raw data using zlib
        std::vector<uint16_t> raw_data = uncompressData(socketData);

        // Push to queue to process raw data
        pushRawDatatoQueue(raw_data);
    }
    close(clientSocket);
    return 0;
}

int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

size_t socketRecvAll(int socket, char *buffer, size_t size) {
    size_t totalReceived = 0;
    while (totalReceived < size) {
        ssize_t bytesReceived = recv(socket, buffer + totalReceived, size - totalReceived, 0);
        if (bytesReceived <= 0) {
            std::cerr << "Receive data failure!" << std::endl;
            return 0;
        }
        totalReceived += bytesReceived;
    }
    return totalReceived;
}

std::vector<uint16_t> uncompressData(const std::vector<uint8_t>& compressedData) {
    size_t decompressedSize = WIDTH * HEIGHT * sizeof(int16_t);
    std::vector<int16_t> deltaFrame(WIDTH * HEIGHT);

    // uncompress zlib
    int result = uncompress(reinterpret_cast<Bytef*>(deltaFrame.data()), &decompressedSize, compressedData.data(), compressedData.size());
    if (result != Z_OK) {
        std::cerr << "Decompression failed! Error code: " << result << std::endl;
        throw std::runtime_error("Decompression failed!");
    }

    // uncompress delta pixel
    std::vector<uint16_t> decoded(WIDTH*HEIGHT);
    decoded[0] = deltaFrame[0];
    for (size_t i = 1; i < WIDTH * HEIGHT; ++i) {
        decoded[i] = deltaFrame[i] + decoded[i - 1];
    }

    return decoded;
}

void pushRawDatatoQueue(std::vector<uint16_t> data){
    pthread_mutex_lock(&data_queue_mutex);
    dataqueue.push(data);
    pthread_mutex_unlock(&data_queue_mutex);
}

int checkRawDataQueue(std::vector<uint16_t> &data){
	int queue_sz = 0;

	pthread_mutex_lock(&data_queue_mutex);
    if(!dataqueue.empty()){
    	queue_sz = dataqueue.size();

    	data = dataqueue.front();
    	dataqueue.pop();   
    }
    pthread_mutex_unlock(&data_queue_mutex);

    return queue_sz;
}

void applyPalette(std::vector<uint16_t> data, cv::Mat& result, std::vector<std::vector<int>> colorPalette){

    for(int i = 0; i < HEIGHT; i++){
        for(int j =0; j < WIDTH; j++){
            int grayValue = mapValue(data[i * WIDTH + j], coldest_point.raw_data, hottest_point.raw_data, 0, 255);
            cv::Vec3b& colorPixel = result.at<cv::Vec3b>(i, j); 

            colorPixel[0] = colorPalette[grayValue][0]; // R
            colorPixel[1] = colorPalette[grayValue][1]; // G
            colorPixel[2] = colorPalette[grayValue][2]; // B 
        }
    }
}

void getTempImage(std::vector<uint16_t> data){
    coldest_point.x = 0;
    coldest_point.y = 0;
    coldest_point.raw_data = 0;
    coldest_point.temp = 150;
    hottest_point.x = 0;
    hottest_point.y = 0;
    hottest_point.raw_data = 0;
    hottest_point.temp = 0;

    // scan all data to get max min temp
    for (int i=0; i<(HEIGHT); i++) {
		for (int j=0; j<WIDTH; j++) {
            float _temp =  float(data[i * WIDTH + j] / 100.0) - 273.15;
            if(coldest_point.temp > _temp){
                coldest_point.temp = _temp;
                coldest_point.x = j;
                coldest_point.y = i;
                coldest_point.raw_data = data[i * WIDTH + j];
            }

            if(hottest_point.temp < _temp){
                hottest_point.temp = _temp;
                hottest_point.x = j;
                hottest_point.y = i;
                hottest_point.raw_data = data[i * WIDTH + j];
            }
            if(i == cursor_point.y && j == cursor_point.x){
                cursor_point.temp = _temp;
            }
        }
    }

    // printf("[Max] Temp: %.2f - Position: %dx%d \n", hottest_point.temp, hottest_point.x, hottest_point.y);
    // printf("[Min] Temp: %.2f - Position: %dx%d \n", coldest_point.temp, coldest_point.x, coldest_point.y);
}

void drawTempPoint(cv::Mat &image, temp_point_t temp_point, cv::Scalar color){
    int thickness = 1;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.4;
    int baseline = 0;
    int x = temp_point.x;
    int y = temp_point.y;

    // draw cursor
    cv::line(image, cv::Point(x + 10, y), cv::Point(x + 5, y), color, thickness, cv::LINE_8);
    cv::line(image, cv::Point(x - 10, y), cv::Point(x - 5, y), color, thickness, cv::LINE_8);
    cv::line(image, cv::Point(x, y + 10), cv::Point(x, y + 5), color, thickness, cv::LINE_8);
    cv::line(image, cv::Point(x, y - 10), cv::Point(x, y - 5), color, thickness, cv::LINE_8);

    // draw temp
    char buf_text[256] = {0};
    sprintf(buf_text, "%.2fC ", temp_point.temp);

    cv::Size textSize = cv::getTextSize(buf_text, fontFace, thickness, fontScale, &baseline);
    baseline += thickness;

    // offset position text 
    if(x <= image.cols/2) x += 5;
    else if(x > image.cols/2) x -= textSize.width/3 + 7;

    if(y < image.rows/2) y += 2*textSize.height/3;
    else if(y >= image.rows/2) y -= 3;

    cv::Point textOrg1(x, y - 1);
    cv::Point textOrg2(x + textSize.width/3 + 2, y - 2*textSize.height/3 + 3);

    cv::putText(image, buf_text, textOrg1, fontFace, fontScale, color, thickness, 8);  
}