#include <cmath>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <iostream>

#define T_PER_BLOCK 16
#define MINF 0
using namespace std;
// cuda的 fabs()； ceil();   gaussD();

inline __device__ float gaussD(float sigma, int x, int y)
{
    return exp(-((x*x + y*y) / (2.0f*sigma*sigma)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Erode Depth Map 对深度图进行腐蚀
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void erodeDepthMapDevice(float* d_output, float* d_input, int structureSize, int width, int height, float dThresh, float fracReq)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;


    if (x >= 0 && x < width && y >= 0 && y < height)
    {


        unsigned int count = 0;

        float oldDepth = d_input[y*width + x];	//获取(x,y)处的深度值，这里是以米为单位的
        for (int i = -structureSize; i <= structureSize; i++)
        {
            for (int j = -structureSize; j <= structureSize; j++)
            {
                if (x + j >= 0 && x + j < width && y + i >= 0 && y + i < height)
                {
                    float depth = d_input[(y + i)*width + (x + j)];		//该像素周围的像素(y+i,x+j)
                    if (depth == MINF || depth == 0.0f || fabs(depth - oldDepth) > dThresh)		//如果该像素跟周围像素差异太大
                    {
                        count++;
                        //d_output[y*width+x] = MINF;
                        //return;
                    }
                }
            }
        }

        unsigned int sum = (2 * structureSize + 1)*(2 * structureSize + 1);
        if ((float)count / (float)sum >= fracReq) {	//如果周围像素跟该像素差异太大，则将当前像素设为0
            d_output[y*width + x] = MINF;
        }
        else {
            d_output[y*width + x] = d_input[y*width + x];	//如果周围像素差异不大，则将当前像素设为原来的值
        }
    }
}

void erodeDepthMap(float* d_output, float* d_input, int structureSize, unsigned int width, unsigned int height, float dThresh, float fracReq)
{
    const dim3 gridSize((width + T_PER_BLOCK - 1) / T_PER_BLOCK, (height + T_PER_BLOCK - 1) / T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    erodeDepthMapDevice << <gridSize, blockSize >> >(d_output, d_input, structureSize, width, height, dThresh, fracReq);
#ifdef _DEBUG
    cutilSafeCall(cudaDeviceSynchronize());
    cutilCheckMsg(__FUNCTION__);
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gauss Filter Float Map
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void gaussFilterDepthMapDevice(float* d_output, const float* d_input, float sigmaD, float sigmaR, unsigned int width, unsigned int height)
{
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= width || y >= height) return;

	const int kernelRadius = (int)ceil(2.0*sigmaD);

	d_output[y*width + x] = MINF;

	float sum = 0.0f;
	float sumWeight = 0.0f;

	const float depthCenter = d_input[y*width + x];
	if (depthCenter != MINF)
	{
		for (int m = x - kernelRadius; m <= x + kernelRadius; m++)
		{
			for (int n = y - kernelRadius; n <= y + kernelRadius; n++)
			{
				if (m >= 0 && n >= 0 && m < width && n < height)
				{
					const float currentDepth = d_input[n*width + m];

					if (currentDepth != MINF && fabs(depthCenter - currentDepth) < sigmaR)
					{
						const float weight = gaussD(sigmaD, m - x, n - y);

						sumWeight += weight;
						sum += weight*currentDepth;
					}
				}
			}
		}
	}

	if (sumWeight > 0.0f) d_output[y*width + x] = sum / sumWeight;
    // printf("%f\n", d_output[y*width + x]);
}

void gaussFilterDepthMap(float* d_output, const float* d_input, float sigmaD, float sigmaR, unsigned int width, unsigned int height)
{
	const dim3 gridSize((width + T_PER_BLOCK - 1) / T_PER_BLOCK, (height + T_PER_BLOCK - 1) / T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	gaussFilterDepthMapDevice << <gridSize, blockSize >> >(d_output, d_input, sigmaD, sigmaR, width, height);
    cout << "1" << endl;
#ifdef _DEBUG
	cutilSafeCall(cudaDeviceSynchronize());
	cutilCheckMsg(__FUNCTION__);
    cout << "1" << endl;

#endif
}

