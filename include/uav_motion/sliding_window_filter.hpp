#pragma once

#include <Eigen/Eigen>
#include <iostream>

// Sliding Window Filter
class SlidingWindowFilter
{
    private:
        int counter, d, ws;
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> data_stack;
        Eigen::Matrix<double,Eigen::Dynamic,1> data_sum, data, data_diff;
    
    public:
        SlidingWindowFilter();
        SlidingWindowFilter(int dimension, int win_size);
        void set_filter(int dimension, int win_size);
        Eigen::Matrix<double,Eigen::Dynamic,1> update(Eigen::Matrix<double,Eigen::Dynamic,1> new_data);
        Eigen::Matrix<double,Eigen::Dynamic,1> update_diff(Eigen::Matrix<double,Eigen::Dynamic,1> new_data);
        Eigen::Matrix<double,Eigen::Dynamic,1> get_data() const;
};

SlidingWindowFilter::SlidingWindowFilter()
{
    this->data_stack.resize(3, 10);
    this->data_sum.resize(3, 1);
    this->data.resize(3, 1);
    this->data_sum.setZero();
    this->data_diff.setZero();
    
    this->counter = 0;
    this->d = 3;
    this->ws = 10;
}

SlidingWindowFilter::SlidingWindowFilter(int dimension, int win_size)
{
    this->data_stack.resize(dimension, win_size);
    this->data_sum.resize(dimension, 1);
    this->data.resize(dimension, 1);
    this->data_sum.setZero();
    this->data_diff.setZero();
    
    this->counter = 0;
    this->d = dimension;
    this->ws = win_size;
}

void SlidingWindowFilter::set_filter(int dimension, int win_size)
{
    this->data_stack.resize(dimension, win_size);
    this->data_sum.resize(dimension, 1);
    this->data.resize(dimension, 1);
    this->data_sum.setZero();
    this->data_diff.setZero();
    
    this->counter = 0;
    this->d = dimension;
    this->ws = win_size;
}

Eigen::Matrix<double,Eigen::Dynamic,1> SlidingWindowFilter::update(Eigen::Matrix<double,Eigen::Dynamic,1> new_data)
{
    
    if (this->counter < this->ws)
    {
        this->data_sum.noalias() = this->data_sum + new_data;
        this->data_stack.block(0,this->counter,this->d,1).noalias() = new_data;
        this->counter = this->counter + 1;
        this->data.noalias() = this->data_sum / this->counter;
    }
    else
    {
        this->data_sum.noalias() = this->data_sum - this->data_stack.block(0,0,this->d,1) + new_data;
        this->data_stack.block(0,0,this->d,this->ws-1).noalias() = this->data_stack.block(0,1,this->d,this->ws-1);
        this->data_stack.block(0,this->ws-1,this->d,1).noalias() = new_data;
        this->data.noalias() = this->data_sum / this->ws;
    }
    return this->data;
}

Eigen::Matrix<double,Eigen::Dynamic,1> SlidingWindowFilter::update_diff(Eigen::Matrix<double,Eigen::Dynamic,1> new_data)
{
    if (this->counter < this->ws)
    {
        this->data_stack.block(0,this->counter,this->d,1).noalias() = new_data;
        this->counter = this->counter + 1;
        this->data_diff = new_data - this->data_stack.block(0,0,this->d,1);
    }
    else{
        this->data_stack.block(0,0,this->d,this->ws-1).noalias() = this->data_stack.block(0,1,this->d,this->ws-1);
        this->data_stack.block(0,this->ws-1,this->d,1).noalias() = new_data;
        this->data_diff = new_data - this->data_stack.block(0,0,this->d,1);
    }
    return this->data_diff;
}


Eigen::Matrix<double,Eigen::Dynamic,1> SlidingWindowFilter::get_data() const { return this->data; }