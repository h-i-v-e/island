/*
 Lifted from https://imgur.com/a/L7rsM
 
 Here is a single iteration. So let me run down the basic algorithm. It's ind of like a mutant Worley.
 
 Start with a heightmap. Now, for a grid of randomly offset points (though poisson disk sampling might look better), have each point 'connect' to its lowest neighbor.
 
 Once the connections are set, cycle every pixel. Each pixel cycles all nearby points and evaluates the equations f1=((y1-y2)*(y-y1)+(x1-x2)*(x-x1))/(sqr(y1-y2)+sqr(x1-x2)) and f2=abs(((y1-y2)*(x-x1)-(x1-x2)*(y-y1))/sqrt(sqr(x1-x2)+sqr(y1-y2))), where (x1,y1) are the coordinates of the point being checked, (x2,y2) are the coordinates of the point that is connected to, and (x,y) are the coordinates of the pixel.
 
 If f1>0, the height from that point is sqrt(sqr(x-x1))+sqr(y-y1)). If f1<-1, the height from that point is sqrt(sqr(x-x2))+sqr(y-y2)). If 0>f1>-1, the height from that point is f2. The final height of the pixel is the minimum height given from all points. Iterate and combine as you please.

 */
 
 

#ifndef yankee_minstrel_erosian_h
#define yankee_minstrel_erosian_h


#endif /* yankee_minstrel_erosian_h */
