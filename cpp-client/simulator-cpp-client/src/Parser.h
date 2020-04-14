#include "Buffer.h"
#include "JsonHelpers.h"
#include <algorithm>
#include "Configuration.h"

class DataFrame{
public:
    virtual void parse(ByteBuffer* buffer) = 0;
};

// for now 8 bit only, todo: add float higher bit rate etc enum
class ImageFrame : DataFrame{
public:
    ImageFrame(int x_res, int y_res) : channels(channels){
        resolution.x = x_res;
        resolution.y = y_res;
        data = new uint8_t[channels * resolution.x * resolution.y];
    }
    ~ImageFrame(){
        delete data;
    }
    uint8_t* data;
    int channels;
    struct Resolution{
        int x;
        int y;
    } resolution;
    int size(){
        return resolution.x * resolution.y * channels;
    }
    virtual void parse(ByteBuffer& buffer){
        std::copy(buffer->data(), buffer()+, );
    }
};

class AnnotationFrame : DataFrame{
public:
    nlohmann::json data;
};

class RadarFrame : DataFrame{
};