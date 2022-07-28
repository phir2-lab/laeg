/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Vitor Jorge
 * For more information see <https://github.com/phir2-lab/laeg>
 *
 * LAEG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LAEG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LAEG If not, see <https://www.gnu.org/licenses/>.
**/

#include "thinning.h"

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

Thinning::Thinning()
{
}
/**** Thninning algorithm version 2*
*/

//Grab the patterns for the thinning algorithm
void Thinning::_grabThinPattern2(const unsigned char *image, int width, int center, specialThinPattern &p)
{
    p.pattern[1]=image[center+width-1];	// position 0
    p.pattern[2]=image[center+width  ];	// position 1
    p.pattern[3]=image[center+width+1];	// position 2
    p.pattern[4]=image[center      +1];	// position 3
    p.pattern[5]=image[center-width+1];	// position 4
    p.pattern[6]=image[center-width  ];	// position 5
    p.pattern[7]=image[center-width-1];	// position 6
    p.pattern[8]=image[center      -1];	// position 7
}

//XH calculated through NC8 -> this is G1 condition
int Thinning::_XH(const specialThinPattern &p)
{
    bool p1 = p.pattern[1] == 0xFF ? true : false;
    bool p2 = p.pattern[2] == 0xFF ? true : false;
    bool p3 = p.pattern[3] == 0xFF ? true : false;
    bool p4 = p.pattern[4] == 0xFF ? true : false;
    bool p5 = p.pattern[5] == 0xFF ? true : false;
    bool p6 = p.pattern[6] == 0xFF ? true : false;
    bool p7 = p.pattern[7] == 0xFF ? true : false;
    bool p8 = p.pattern[8] == 0xFF ? true : false;

    int count= int((!p2)&&(p3||p4))+int((!p4)&&(p5||p6))+int((!p6)&&(p7||p8))+int((!p8)&&(p1||p2));

    return count;
}

//G2 condition
int Thinning::_minN1N2(const specialThinPattern &p)
{
    int N1=0, N2=0;

    bool p1 = p.pattern[1] == 0xFF ? true : false;
    bool p2 = p.pattern[2] == 0xFF ? true : false;
    bool p3 = p.pattern[3] == 0xFF ? true : false;
    bool p4 = p.pattern[4] == 0xFF ? true : false;
    bool p5 = p.pattern[5] == 0xFF ? true : false;
    bool p6 = p.pattern[6] == 0xFF ? true : false;
    bool p7 = p.pattern[7] == 0xFF ? true : false;
    bool p8 = p.pattern[8] == 0xFF ? true : false;

    //uniformizado para paper Guo
    //The N2 pattern
    N1= int(p1 || p2) + int(p3 || p4) + int(p5 || p6) + int(p7 || p8);
    //The N2 pattern
    N2=	int(p2 || p3) + int(p4 || p5) + int(p6 || p7) + int(p8 || p1);

    return min(N1, N2);
}

bool Thinning::_testG3_odd(const specialThinPattern &p)//odd  condition
{
    bool p2 = p.pattern[2] == 0xFF ? true : false;
    bool p3 = p.pattern[3] == 0xFF ? true : false;
    bool p4 = p.pattern[4] == 0xFF ? true : false;
    bool p5 = p.pattern[5] == 0xFF ? true : false;

    return (p2||p3||(!p5)) && (p4);
}

bool Thinning::_testG3_even(const specialThinPattern &p)//even condition
{
    bool p1 =p.pattern[1] == 0xFF ? true : false;
    bool p6 =p.pattern[6] == 0xFF ? true : false;
    bool p7 =p.pattern[7] == 0xFF ? true : false;
    bool p8 =p.pattern[8] == 0xFF ? true : false;

    return (p6||p7||(!p1)) && (p8);
}

//Thinning algorithm from GUO and Hall
bool Thinning::Thinning_Using_GUOandHALL(const unsigned char* image, unsigned char* output, int width, int height)
{
    if(image==NULL)
    {
        //image is null
        std::cerr << "Received null image to thin." << std::endl;
        return false;
    }

    if(width<3 || height < 3)
        return false;

    //the output image is originally equal the input image
    memcpy(output, image, width*height*sizeof(unsigned char));

    std::vector<int> to_be_processed;

    int turn=0;
    bool iterate=true;
    while(iterate)
    {
        //try to set the last iteration
        iterate=false;

        turn++;

        to_be_processed.clear();

        //start from white patterns
        for(int line = 1; line < height-1; line++)
        {
            for(int col = 1; col < width-1; col++)
            {
                //Grab central position
                int center = width*line+col;

                //Check if it is one
                if(output[center]==0xFF)
                {
                    //declaring the pattern grabber
                    specialThinPattern p;
                    //grabbing the neighborhood
                    _grabThinPattern2(output, width, center, p);

                    //Checking conditions 1 and 2
                    bool G1 = (_XH(p)==1) ?  true : false;

                    int N =_minN1N2(p);
                    bool G2 = ((2 <= N) && (N <= 3)) ? true : false;

                    bool G3;
                    if(turn%2==0)
                        G3 = _testG3_even(p);//even
                    else
                        G3 = _testG3_odd(p); //odd

                    //If conditions are satisfied erase pixel
                    if((G1==true) && (G2==true) && (G3==false))
                    {
                        to_be_processed.push_back(center);

                        iterate=true;
                    }

                }
            }
        }

        //Now delete pixels
        for(size_t pixel = 0; pixel < to_be_processed.size(); pixel++)
            output[to_be_processed[pixel]] = 0x00;

    }

    _setBordersAsZero(output, width, height);

    return true;
}

void Thinning::_setBordersAsZero(unsigned char *image, const size_t& width, const size_t& height)
{
    //Setting lower border
    int numPixelsLine=width;
    for(int i=0;i<numPixelsLine;i++)
    {
        image[i]=0x00;
    }
    //discovering upper border
    int last=width*height-1;
    int fol=last - numPixelsLine;
    for (int i=fol;i<last;i++)
    {
        image[i]=0x00;
    }
    //Setting left border
    for(int i=0;i<last;i+=numPixelsLine)
    {
        image[i]=0x00;
    }
    //Setting right border
    for(int i=numPixelsLine-3;i<last;i+=numPixelsLine)
    {
        image[i]=0x00;
    }
}

bool Thinning::spur_removal(int iterations, unsigned char* image, int width, int height)
{
    if(image==NULL)
    {
        //image is null
        std::cerr << "Received null image to thin." << std::endl;
        return false;
    }

    if(width<3 || height < 3)
        return false;

    int offset[][8]={{-1,  1},
                     { 0,  1},
                     { 1,  1},
                     { 1,  0},
                     { 1, -1},
                     { 0, -1},
                     {-1, -1},
                     {-1,  0}};

    //list of possible spur segments
    vector< vector<int> > possibleSpurs;

    stack<int> st;
    int intercection[8]={-1, -1, -1, -1, -1, -1, -1, -1};
    //start from white patterns
    for(int line = 1; line < height-1; line++)
    {
        for(int col = 1; col < width-1; col++)
        {
            //temporary storage of the tip of the spur
            vector<int> ending;

            //Grab central position
            int center=width*line+col;
            //Check if the current point is thinning point
            if(image[center]==0xFF)
            {
                int counting=0;
                for(int i=0;i<8;++i)
                    if(image[center+offset[i][1]*width+offset[i][0]]==0xFF)
                    {
                        counting++;
                    }

                //Add thinning edge point to the list
                if(counting==1)
                {
                    st.push(center);
                }

                // isolated thinning point removal
                if(counting==0)
                {
                    image[center  ]=0x00;
                }
            }
        }
    }

    // Remove spurious edge lines up to a predefined limit
    while(!st.empty())
    {
        int center = st.top();
        st.pop();

        int iter=0;
        while(iter<iterations)
        {
            int neighbor;
            int counting=0;
            for(int n=0;n<8;++n)
            {
                if(image[center+offset[n][1]*width-offset[n][0]]==0xFF)
                {
                    neighbor=center+offset[n][1]*width-offset[n][0];
                    counting++;
                }
            }
            // Remove the thinning point only if it is still a edge point
            // (I guess this removes thinning discontinuities)
            if(counting==1)
            {
                if(image[center]==0xFF)
                {
                    image[center]=0x00;
                    center=neighbor;
                }
            }else{
                break;
            }
            iter++;
        } 
    }

   return true; 
}
