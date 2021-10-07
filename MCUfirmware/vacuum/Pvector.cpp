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

#include "PVector.h"

PVector::PVector() {
  x = y = z = 0.0;
}

PVector::PVector(float _x, float _y, float _z) {
  x = _x; y = _y; z = _z;
}

void PVector::add(PVector p) {
  x += p.x;
  y += p.y;
  z += p.z;
}

void PVector::normalize() {
  float l = sqrt(x * x + y * y + z * z);
  x /= l;
  y /= l;
  z /= l;
}

float PVector::dot(PVector p) {
  return x * p.x + y * p.y + z * p.z;
}

float PVector::dist(PVector p) {
  float dist_x = x - p.x;
  float dist_y = y - p.y;
  float dist_z = z - p.z;
  return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}

