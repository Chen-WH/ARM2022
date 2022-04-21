//
// Created by zm on 18-12-1.
//

#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>

class Parameter {

public:
    float pos_x;
    float pos_y;
    float pos_z;

    float x_p;
    float y_p;
    float z_p;

    float vx_p;
    float vy_p;
    float vz_p;

    float vx_i;
    float vy_i;
    float vz_i;

    float vx_d;
    float vy_d;
    float vz_d;

    float txp_p;
    float txp_i;
    float txp_d;
    float txv_p;
    float txv_i;
    float txv_d;

    float typ_p;
    float typ_i;
    float typ_d;
    float tyv_p;
    float tyv_i;
    float tyv_d;

    float tzp_p;
    float tzp_i;
    float tzp_d;
    float tzv_p;
    float tzv_i;
    float tzv_d;

    bool readParam(const char* addr);

};

bool Parameter::readParam(const char *addr) {

    std::ifstream fs;

    std::string name = "";
    float value[3];

    fs.open(addr);

    if (!fs)
    {
        std::cout << "parameter file err" << std::endl;
        return false;
    }

    while (!fs.eof())
    {
        fs >> name >> value[0] >> value[1] >> value[2];

        if (name == "POS")
        {
            pos_x = value[0];
            pos_y = value[1];
            pos_z = value[2];
        }

        if (name == "P")
        {
            x_p = value[0];
            y_p = value[1];
            z_p = value[2];
        }
        if (name == "vP")
        {
            vx_p = value[0];
            vy_p = value[1];
            vz_p = value[2];

        }
        if(name == "vI")
        {
            vx_i = value[0];
            vy_i = value[1];
            vz_i = value[2];

        }
        if (name == "vD")
        {
            vx_d = value[0];
            vy_d = value[1];
            vz_d = value[2];

        }

        if (name == "Tkpp")
        {
            txp_p = value[0];
            typ_p = value[1];
            tzp_p = value[2];

        }
        if (name == "Tkpi")
        {
            txp_i = value[0];
            typ_i = value[1];
            tzp_i = value[2];

        }
        if (name == "Tkpd")
        {
            txp_d = value[0];
            typ_d = value[1];
            tzp_d = value[2];

        }
        if (name == "Tkvp")
        {
            txv_p = value[0];
            tyv_p = value[1];
            tzv_p = value[2];

        }
        if (name == "Tkvi")
        {
            txv_i = value[0];
            tyv_i = value[1];
            tzv_i = value[2];

        }
        if (name == "Tkvd")
        {
            txv_d = value[0];
            tyv_d = value[1];
            tzv_d = value[2];
        }
    }
    std::cout << "read config file successfully!"<<std::endl;

    fs.close();

    return true;
}
