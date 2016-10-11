/*
 * VIOLine.hpp
 *
 *  Created on: Sep 29, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOLINE_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOLINE_HPP_

#include <vector>

using namespace std;

class VIOLine
{

private:
    vector<double> p0; // the position of the camera
    vector<double> p1; // the position the feature one meter away from the camera

public:
    VIOLine() {

    }

    VIOLine(vector<double> x, vector<double> y) {
        this->p0 = x;
        this->p1 = y;
    }

    vector<double> direction_vector() {
        vector<double> a;
        a.push_back(this->p1[2]-this->p0[2]); // z-component
        a.push_back(this->p1[1]-this->p0[1]); // y-component
        a.push_back(this->p1[0]-this->p0[0]); // x-component
        return a;
    }

    //<a1,a2,a3> x <b1, b2, b3> = < (a2b3-a3b2), (a3b1-a1b3), (a1b2-a2b1) >
    vector<double> cross(vector<double> x, vector<double> y) {
        vector<double> z;
        z.push_back((x[0]*y[1]) - (x[1]*y[0])); //z component
        z.push_back((x[2]*y[0]) - (x[0]*y[2])); //y component
        z.push_back((x[1]*y[2]) - (x[2]*y[1])); //x component
        return z;
    }

    double dot(vector<double> x, vector<double> y) {
        return (x[2]*y[2] + x[1]*y[1] + x[0]*y[0]);
    }

    vector<double> add (vector<double> x, vector<double> y) {
        vector<double> a;
        a.push_back(x[2]+y[2]); // z-component
        a.push_back(x[1]+y[1]); // y-component
        a.push_back(x[0]+y[0]); // x-component
        return a;
    }

    vector<double> multiply (vector<double> x, double c) {
        vector<double> a;
        a.push_back(x[2]*c); // z-component
        a.push_back(x[1]*c); // y-component
        a.push_back(x[0]*c); // x-component
        return a;
    }

    vector<double> subtract (vector<double> x, vector<double> y) {
        vector<double> a;
        a.push_back(x[2]-y[2]); // z-component
        a.push_back(x[1]-y[1]); // y-component
        a.push_back(x[0]-y[0]); // x-component
        return a;
    }

    vector<double> cross() {
        vector<double> z;
        z.push_back((this->p0[0]*this->p1[1]) - (this->p0[1]*this->p1[0])); //z component
        z.push_back((this->p0[2]*this->p1[0]) - (this->p0[0]*this->p1[2])); //y component
        z.push_back((this->p0[1]*this->p1[2]) - (this->p0[2]*this->p1[1])); //x component
        return z;
    }

    /*
     * accepts two lines and will find the closest point between those two
     * lines.
     * cp is the closest equidistant point bewteen the two lines
     * returns: the distance at the closest point bewteen the two lines.
     */
    double closest_distance(VIOLine L1, VIOLine L2, vector<double>& cp) {
        vector<double> d1 = (L1.direction_vector());
        vector<double> d2 = (L2.direction_vector());

        vector<double> p1 = L1.getP0();
        vector<double> p2 = L2.getP0();

        vector<double> n = cross(d1, d2);
        vector<double> n1 = cross(d1, n);
        vector<double> n2 = cross(d2, n);

        double frac1 = ((dot(subtract(p2, p1), n2))/(dot(d1, n2)));
        double frac2 = (dot(subtract(p1, p2), n1)/dot(d2, n1));

        vector<double> c1 = add(p1, multiply(d1, frac1));
        vector<double> c2 = add(p2, multiply(d2, frac2));

        cp[0] = (c2[0]+c1[0])/2;
        cp[1] = (c2[1]+c1[1])/2;
        cp[2] = (c2[2]+c1[2])/2;

        vector<double> c21 = subtract(c2, c1);

        return sqrt(c21[0] * c21[0] + c21[1] * c21[1] + c21[2] * c21[2]);
    }

    vector<double> getP0() {
        return this->p0;
    }

    vector<double> getP1() {
        return this->p1;
    }

};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOLINE_HPP_ */
