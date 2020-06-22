#ifndef CENTROID_H
#define CENTROID_H

#include <nlohmann_json/json.h>
#include <complex>
#include <vector>

struct Centroid {
    std::vector<std::complex<double>> centroid_vector;
    float x,y;
    int count;

    nlohmann::json seriallize() {
        nlohmann::json j;
        j["x"] = x;
        j["y"] = y;
        std::vector<double> real, imag;
        for(int i = 0 ; i < centroid_vector.size(); i++) {
            real.push_back(centroid_vector[i].real());
            imag.push_back(centroid_vector[i].imag());
        }
        j["features_real"] = real;
        j["features_imag"] = imag;
        return j;
    }

    static Centroid deserialize(nlohmann::json& j) {
        Centroid cen;
        cen.x = j["x"];
        cen.y = j["y"];
        cen.count = 1;

        assert( j["features_real"].size() == j["features_imag"].size() );

        for(int i = 0; i < j["features_real"].size(); i++) {
            cen.centroid_vector.push_back(std::complex<double>(j["features_real"][i], j["features_imag"][i]));
        }

        return cen;
    }
};

#endif