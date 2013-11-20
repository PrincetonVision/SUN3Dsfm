SUN3Dsfm
==============

How to install
--------------

- You need to install ceres-solver in order to do the bundle adjustment: https://code.google.com/p/ceres-solver/

The current code is tested using version 1.7.0, that you can download from:
https://code.google.com/p/ceres-solver/downloads/detail?name=ceres-solver-1.7.0.tar.gz&can=1&q=

- Compilation

After installing ceres-solver, please cd into the directory, and then compile our code by one of the following two ways.

1. Compile via cmake:
	cmake .
	make

2. Compile via compile.sh:
	./compile.sh


How to run
--------------

There are two main functions RGBDsfm.m and ObjectAdjustment.m

- RGBDsfm is used to run SFM on an RGB-D sequence.

- ObjectAdjustment is used to run the Generalized Bundle Adjustment using semantic object anotaiton. It is required to run RGBDsfm first before running this ObjectAdjustment.

Our typical work process is to run RGBDsfm to get the pose estimations automatically, and then use our online object annotator to annotate the sequence, and finally run ObjectAdjustment again to get a better pose estimation.

How to run on SUN3D
--------------

To run RGBDsfm and ObjectAdjustment, you just need to input a sequence name from the SUN3D database. A full list of name is provided in the project website: http://sun3d.cs.princeton.edu

Examples:
	RGBDsfm('hotel_umd/maryland_hotel3');
	ObjectAdjustment('hotel_umd/maryland_hotel3');

In the code RGBDsfm.m and ObjectAdjustment.m, there is two variables pointing to where the input and output paths are. You can set it to your local directory if you download the data in your own disk, e.g.:
	write2path = '~/user_folder/sun3d/data/sfm/';
	SUN3Dpath = '~/user_folder/fs/sun3d/data/scene/scene_final/';
For the input folder, you can also directly load the data from our server directly, by setting it to be:
	SUN3Dpath = 'http://sun3d.csail.mit.edu/data/';

How to run on your own sequences
--------------

You need to change the IO functions in order to load your own sequences based on your own format. We suggest you convert your own sequences into our format, as we experimented with it a lot and find that it is very reliable and easy to understand.

Our depth file format
--------------

- We assume the depths and the images are pre-algined.
- We save the depth as interger in millimeter = 0.001 meter.
- We use 16-bit PNG file to save the depth.
- We circularly shift 3 bit in the PNG file so that the depth image look nice in a typical image viewer. (otherwise, it will be too dark to see anything)
- Therefore, in the code, during data loading, we have to shift the 3 bit back.

Testing environment
--------------
We have tested our code on Mac and Linux using Matlab 2013a. We haven't tested it in any Windows machine.

What is the algorithm?
--------------

Please read our paper:
J. Xiao, A. Owens and A. Torralba
SUN3D: A Database of Big Spaces Reconstructed using SfM and Object Labels
Proceedings of 14th IEEE International Conference on Computer Vision (ICCV2013)
http://vision.princeton.edu/projects/2013/SUN3D/paper.pdf

Disclaimer
--------------

This is a post-deadline implementation that we don't guarantee to produce exactly the same results as in the paper. And we will try to improve the performance from time to time, and this code will be updated in the future. 

Citation
--------------

You must cite the following paper if you use this code in any ways:

@inproceedings{SUN3D,
	author     = {Jianxiong Xiao and Andrew Owens and Antonio Torralba},
	title      = {{SUN3D}: A Database of Big Spaces Reconstructed using SfM and Object Labels},
	booktitle  = {Proceedings of IEEE International Conference on Computer Vision (ICCV)},
	year       = {2013},
}


License
--------------

It is released under MIT license. All the 3rd party software included are for the convenient of the users. The users must follow their original licenses and copyrights.

Copyright (c) 2013 Jianxiong Xiao

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Questions?
--------------

Email Jianxiong Xiao http://vision.princeton.edu/

