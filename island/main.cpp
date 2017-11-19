//
//  main.cpp
//  island
//
//  Created by Jerome Johnson on 19/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include <iostream>
#include "continent.h"
#include "lodepng.h"
#include "raster.h"

using namespace worldmaker;

void writePNG(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
{
    std::vector<unsigned char> png;
    unsigned error = lodepng::encode(png, image, width, height, LCT_RGB);
    if(!error) lodepng::save_file(png, filename);
    
    //if there's an error,display it
    if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}

int main(int argc, const char * argv[]) {
    Continent continent(666, 4096, 2, 0.1f);
    continent.generateSeasAndLakes(0.5f);
    Raster raster(1024, 1024);
    raster.fill(0xffffffff);
    continent.draw(raster);
    int rasterLength = raster.length() * 3;
    std::vector<unsigned char> buffer;
    buffer.reserve(rasterLength);
    const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
    for (int i = 0, j = raster.length() << 2; i != j; i += 4){
        buffer.push_back(raw[i]);
        buffer.push_back(raw[i + 1]);
        buffer.push_back(raw[i + 2]);
    }
    writePNG("test.png", buffer, 1024, 1024);
    return 0;
}
