#ifdef __CUDACC__
#define BOTH __host__ __device__
#else
#define BOTH 
#endif