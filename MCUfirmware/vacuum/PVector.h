/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: 26 April 2016
/* 
/* Version 1.00
/* www.7bot.cc
/*  
/* Description: 
/* 
/*
/***************************************************/

#ifndef _PVECTOR_H
#define _PVECTOR_H

#include "Arduino.h"


class PVector {
  public:
    /* Global Variables */
    float x, y, z;
    // constructor
    PVector();
    PVector(float _x, float _y, float _z);
    // function
    void add(PVector p);
    void normalize();
    float dot(PVector p);
    float dist(PVector p);

  private:

};

#endif

