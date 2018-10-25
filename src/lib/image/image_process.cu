#include <cuda_runtime.h>
#include <cublas.h>

__global__ void yuyv2yuv_kernal(unsigned char *in, unsigned char *out, unsigned int w)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    out[tmp*3+dst_offset+0] = tp[src_offset+0];
    out[tmp*3+dst_offset+1] = tp[src_offset+(int)pow(-1, x&1)];
    out[tmp*3+dst_offset+2] = tp[src_offset+2+(int)pow(-1, x&1)];
}

template <typename T>
__device__ T boundrgb(T v)
{
    if(v>255) return 255;
    if(v<0) return 0;
    return v;
}

template<typename T>
__device__ T cmin(T v1, T v2)
{
    return v1<v2?v1:v2;
}

template<typename T>
__device__ T cmax(T v1, T v2)
{
    return v1>v2?v1:v2;
}

__global__ void yuyv2bgr_kernal(unsigned char *in, unsigned char *out, unsigned int w)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];
    int r,g,b;

    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<int>(r);
    g = boundrgb<int>(g);
    b = boundrgb<int>(b);

    out[tmp*3+dst_offset+2] = (unsigned char)(r);
    out[tmp*3+dst_offset+1] = (unsigned char)(g);
    out[tmp*3+dst_offset+0] = (unsigned char)(b);
}

__global__ void yuyv2dst_kernal(unsigned char *in, unsigned char *bgr, float *rgb, float *hsi, unsigned int w, unsigned int h)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int planesize = w*h;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    //YUYV2YUV
    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];

    //YUV2BGR
    float r,g,b;
    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<float>(r);
    g = boundrgb<float>(g);
    b = boundrgb<float>(b);

    float bn = b/255.0f;
    float gn = g/255.0f;
    float rn = r/255.0f;

    // BGR2HSI
    float min_v = cmin<float>(bn, cmin<float>(gn, rn));
    float H, S, I;
    float eps = 0.000001;
    I = (bn+gn+rn)/3.0f+eps;
    S = 1.0f-min_v/I;

    H = acos(0.5*(rn-gn+rn-bn)/sqrt((rn-gn)*(rn-gn)+(rn-bn)*(gn-bn)+eps));
    if(bn>gn) H = 2*M_PI-H;
    hsi[tmp+x] = H*180.0f/M_PI;
    hsi[planesize+tmp+x] = S*100.0f;
    hsi[2*planesize+tmp+x] = I*100.0f;

    bgr[tmp*3+dst_offset+0] = (unsigned char)b;
    bgr[tmp*3+dst_offset+1] = (unsigned char)g;
    bgr[tmp*3+dst_offset+2] = (unsigned char)r;

    rgb[tmp+x] = rn;
    rgb[planesize+tmp+x] = gn;
    rgb[2*planesize+tmp+x] = bn;
}


__global__ void resizew_kernal(float *src, int iw, int ih, float *dst, int ow, int oh)
{
    int y = blockIdx.x;
    int x = threadIdx.x;
    int planesizeo = ow*oh;
    int planesizei = iw*ih;
    int tmpo = y*ow;
    int tmpi = y*iw;
    float w_scale = (float)(iw-1)/(ow-1);
    float sx = x*w_scale;
    int ix = (int) sx;
    float dx = sx - ix;
    if(ix<iw-1)
    {
        dst[tmpo+x] = (1-dx)*src[tmpi+ix]+dx*src[tmpi+ix+1];
        dst[planesizeo+tmpo+x] = (1-dx)*src[planesizei+tmpi+ix]+dx*src[planesizei+tmpi+ix+1];
        dst[2*planesizeo+tmpo+x] = (1-dx)*src[2*planesizei+tmpi+ix]+dx*src[2*planesizei+tmpi+ix+1];
    }
}

__global__ void resizeh_kernal(float *src, int iw, int ih, float *dst, int ow, int oh)
{
    int y = blockIdx.x;
    int x = threadIdx.x;
    int planesizeo = ow*oh;
    int planesizei = iw*ih;
    int tmpo = y*ow;
    float h_scale = (float)(ih-1)/(oh-1);

    float sy = y*h_scale;
    int iy = (int) sy;
    float dy = sy - iy;
    int tmpi1 = iy*iw;
    int tmpi2 = (iy+1)*iw;
    if(iy<ih-1)
    {
        dst[tmpo+x] = (1-dy)*src[tmpi1+x]+dy*src[tmpi2+x];
        dst[planesizeo+tmpo+x] = (1-dy)*src[planesizei+tmpi1+x]+dy*src[planesizei+tmpi2+x];
        dst[2*planesizeo+tmpo+x] = (1-dy)*src[2*planesizei+tmpi1+x]+dy*src[2*planesizei+tmpi2+x];
    }
}

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2yuv_kernal<<<h, w, w*2>>>(in,out,w);
}

void cudaYUYV2BGR(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2bgr_kernal<<<h, w, w*2>>>(in,out,w);
}

void cudaYUYV2DST(unsigned char *in, unsigned char *bgr, float *rgb, float *hsi, const unsigned int &w, const unsigned int &h)
{
    yuyv2dst_kernal<<<h,w,w*2>>>(in, bgr, rgb, hsi, w, h);
}

void cudaResize(float *in, int iw, int ih, float *sizedw, float *sized, int ow, int oh)
{
    resizew_kernal<<<ih, ow>>>(in, iw, ih, sizedw, ow, ih);
    resizeh_kernal<<<oh, ow>>>(sizedw, ow, ih, sized, ow, oh);
}