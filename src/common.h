#ifndef COMMON_H
#define COMMON_H



#define TEXTURE_WIDTH 1024
#define VIDEO_WIDTH 360
#define VIDEO_HEIGHT 240

typedef struct {
	int frameNum;
	uint32_t pixels [VIDEO_WIDTH*VIDEO_HEIGHT];
} SharedState;



#endif // COMMON_H
