#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "api.cpp"

using namespace cv;
using namespace std;

char serialPortFilename[] = "/dev/ttyACM0";

void writeToArduino(FILE *serPort, char *command);
void readFromArduino(FILE *serPort, uint16_t *data, int size);
void getEeprom(FILE *serPort, uint16_t *eedata1, uint16_t *eedata2);
void getFrame(FILE *serPort, uint16_t *framedata1, uint16_t *framedata2);
void fullFrame(uint16_t *curFrame, float *curResult, Mat img);
void halfFrame(uint16_t *curFrame, float *curResult, Mat img, int cam);


paramsMLX90640 params1;
paramsMLX90640 params2;
FILE *serPort;

/**
 * Opens serial port and restarts arduino, reads eeprom data and 
 * extracts paramteres.
 */

void setup() {
	uint16_t eedata1[1000];
	uint16_t eedata2[1000];
	memset(eedata1, 0, sizeof(eedata1));
	memset(eedata2, 0, sizeof(eedata2));
	printf("opening serial port and restarting arduino\n");
	serPort = fopen(serialPortFilename, "a+rw");
	printf("wait for arduino to restart\n");
	fflush(stdout);
	sleep(3);
	printf("getting eeprom data");
	getEeprom(serPort, eedata1, eedata2);
	printf("Last numbers: %d, %d\n", eedata1[831], eedata2[831]);
	MLX90640_ExtractParameters(eedata1, &params1);
	MLX90640_ExtractParameters(eedata2, &params2);
	printf("Vdd: %d", params1.kVdd);
	printf("vdd25: %d", params1.vdd25);
	printf("KvPTAT: %f\n", params1.KvPTAT);
	printf("Vdd: %d", params2.kVdd);
	printf("vdd25: %d", params2.vdd25);
	printf("KvPTAT: %f\n", params2.KvPTAT);	
	fflush(stdout);
	sleep(1);
}

void getThermalImage(Mat img1, Mat img2) {
	cout << "Getting thermal image" << endl;	
	uint16_t curFrame1[1000];
	float curResult1[1000];
	uint16_t curFrame2[1000];
	float curResult2[1000];
	memset(curFrame1, 0, sizeof(curFrame1));
	memset(curFrame2, 0, sizeof(curFrame2));
	getFrame(serPort, curFrame1, curFrame2);
	cout << "Processing data" << endl;
	MLX90640_CalculateTo(curFrame1, &params1, curResult1);
	MLX90640_CalculateTo(curFrame2, &params2, curResult2);
	cout << "correcting data" << endl;
	MLX90640_BadPixelsCorrection((&params1)->brokenPixels, curResult1, 1, &params1);
	MLX90640_BadPixelsCorrection((&params1)->outlierPixels, curResult1, 1, &params1);
	MLX90640_BadPixelsCorrection((&params2)->brokenPixels, curResult2, 1, &params2);
	MLX90640_BadPixelsCorrection((&params2)->outlierPixels, curResult2, 1, &params2);
	cout << "converting to img" << endl;
	halfFrame(curFrame1, curResult1, img1, 1);	
	halfFrame(curFrame2, curResult2, img2, 2);
}
/**
int main(int argc, char** argv) {
	uint16_t eedata1[1000];
	uint16_t eedata2[1000];
	memset(eedata1, 0, sizeof(eedata1));
	memset(eedata2, 0, sizeof(eedata2));
	printf("opening serial port and restarting arduino\n");
	FILE *serPort = fopen(serialPortFilename, "a+rw");
	printf("wait for arduino to restart\n");
	fflush(stdout);
	sleep(3);
	printf("getting eeprom data");
	getEeprom(serPort, eedata1, eedata2);
	printf("Last numbers: %d, %d\n", eedata1[831], eedata2[831]);
	MLX90640_ExtractParameters(eedata1, &params1);
	MLX90640_ExtractParameters(eedata2, &params2);
	printf("Vdd: %d", params1.kVdd);
	printf("vdd25: %d", params1.vdd25);
	printf("KvPTAT: %f\n", params1.KvPTAT);
	printf("Vdd: %d", params2.kVdd);
	printf("vdd25: %d", params2.vdd25);
	printf("KvPTAT: %f\n", params2.KvPTAT);	
	fflush(stdout);
	sleep(1);
	uint16_t curFrame1[1000];
	float curResult1[1000];
	uint16_t curFrame2[1000];
	float curResult2[1000];
	Mat img1(160, 120, CV_8UC3, Scalar(126, 0, 255));
	Mat img2(160, 120, CV_8UC3, Scalar(126, 0, 255));
	imshow("test1", img1);
	imshow("test2", img2);
	waitKey(1);
	while(true) {
		memset(curFrame1, 0, sizeof(curFrame1));
		memset(curFrame2, 0, sizeof(curFrame2));
		getFrame(serPort, curFrame1, curFrame2);
		MLX90640_CalculateTo(curFrame1, &params1, curResult1);
		MLX90640_CalculateTo(curFrame2, &params2, curResult2);
		MLX90640_BadPixelsCorrection((&params1)->brokenPixels, curResult1, 1, &params1);
		MLX90640_BadPixelsCorrection((&params1)->outlierPixels, curResult1, 1, &params1);
		MLX90640_BadPixelsCorrection((&params2)->brokenPixels, curResult2, 1, &params2);
		MLX90640_BadPixelsCorrection((&params2)->outlierPixels, curResult2, 1, &params2);
		halfFrame(curFrame1, curResult1, img1, 1);	
		halfFrame(curFrame2, curResult2, img2, 2);
	}
}
**/
void fullFrame(uint16_t *curFrame, float *curResult, Mat img) {
	cout << "Subpage " << curFrame[833] << endl;
	for (int i = 0; i < 24; i++) {
		cout << endl;
		for (int j = 0; j < 32; j++) {
			float pixVal = curResult[i*24 + j];
			if (pixVal > 10 && pixVal < 300) {
				cout << setprecision(2) << curResult[i*24 + j] << "|";
			} else {
				cout << "e" << "|";
			}
			Vec3b col;
			col[0] = 0;
			col[1] = 0;
			col[2] = (curResult[i*24+j] > 0) ? curResult[i*24+j] * 3 : 0;
			for (int x = 0; x < 10; x++) {
				for (int y = 0; y < 10; y++) {
					img.at<Vec3b>(Point(i*10+x, j*10+y)) = col;
				}
			}
		}
	}
	cout << endl;
	imshow("test", img);
	waitKey(1);
}


void halfFrame(uint16_t *curFrame, float *curResult, Mat img, int cam) {
	cout << "Subpage " << curFrame[833];
	int subpage = curFrame[833];
	if (subpage > 1 || subpage < 0) {
		cout << "Error" << endl;
		return;
	}
	for (int i = 0; i < 24; i++) {
		cout << endl;
		for (int j = 0; j < 32; j++) {
			if ((subpage == 0 && (i % 2 == j % 2)) || (subpage == 1 && (i % 2 != j % 2))) {
				float pixVal = curResult[i*32 + j];
				if (pixVal < 0) {
					pixVal = 0;
				}
				cout << setprecision(2) << pixVal << "|";
				Vec3b col;
				col[0] = 0;
				col[1] = 1;
				col[2] = (pixVal * 5 > 255) ? 255 : pixVal * 5;
				for (int x = 0; x < 10; x++) {
					for (int y = 0; y < 10; y++) {
						img.at<Vec3b>(Point((i/2)*10+x, (j/2)*10+y)) = col;
					}
				}	
			}
		}
	}		
	cout << endl;
	/**
	if (cam == 1) {
		imshow("test1", img);
	} else if (cam == 2) {
		imshow("test2", img);
	}
	waitKey(1);
	**/
}

void getEeprom(FILE *serPort, uint16_t *eedata1, uint16_t *eedata2) {
	char writeBuffer[] = {"e"};
	writeToArduino(serPort, writeBuffer);
	readFromArduino(serPort, eedata1, 832);
	readFromArduino(serPort, eedata2, 832);
}

void getFrame(FILE *serPort, uint16_t *framedata1, uint16_t *framedata2) {
	cout << "getting frame data" << endl;
	char writeBuffer[] = {"f"};
	writeToArduino(serPort, writeBuffer);
	cout << "getting frame 1" << endl;
	readFromArduino(serPort, framedata1, 834);
	readFromArduino(serPort, framedata2, 834);
}

void writeToArduino(FILE *serPort, char *command) {
	fwrite(command, sizeof(char), sizeof(command), serPort);
}

void readFromArduino(FILE *serPort, uint16_t *data, int size) {
	printf("getting data\n");
	char readBuffer[10];
	memset(readBuffer, 0, sizeof(readBuffer));
	int sum = 0;
	for (int i = 0; i < size;) {
		if (fread(readBuffer, sizeof(char), 1, serPort) == 1) {
			if (readBuffer[0] > 13) {
				sum *= 10;
				sum += readBuffer[0] - '0';
			}
			if (readBuffer[0] == 13) {
				//printf("%d %d\n", sum, i);
				data[i] = sum;
				sum = 0;
				i++;
			}
			if (readBuffer[0] == 'd') {
				printf("done");
				return;
			}
		}
	}
}

