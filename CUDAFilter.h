#pragma once

void erodeDepthMap(float* d_output, float* d_input, int structureSize, unsigned int width, unsigned int height, float dThresh, float fracReq);

void gaussFilterDepthMap(float* d_output, const float* d_input, float sigmaD, float sigmaR, unsigned int width, unsigned int height);

