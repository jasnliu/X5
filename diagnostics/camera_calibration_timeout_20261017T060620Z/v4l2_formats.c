#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>
int main(int argc, char**argv) {
 int f=open(argv[1],O_RDWR); if(f<0){perror("open");return 1;}
 struct v4l2_format fmt={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};
 if(ioctl(f,VIDIOC_G_FMT,&fmt)==0) printf("current %u %u sizeimage=%u bytesperline=%u fourcc=%c%c%c%c\n",fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.sizeimage,fmt.fmt.pix.bytesperline,fmt.fmt.pix.pixelformat&255,(fmt.fmt.pix.pixelformat>>8)&255,(fmt.fmt.pix.pixelformat>>16)&255,(fmt.fmt.pix.pixelformat>>24)&255);
 for(unsigned i=0;;++i){struct v4l2_fmtdesc d={.index=i,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};if(ioctl(f,VIDIOC_ENUM_FMT,&d)<0)break;printf("format %s %c%c%c%c\n",d.description,d.pixelformat&255,(d.pixelformat>>8)&255,(d.pixelformat>>16)&255,(d.pixelformat>>24)&255);
 for(unsigned j=0;;++j){struct v4l2_frmsizeenum s={.index=j,.pixel_format=d.pixelformat};if(ioctl(f,VIDIOC_ENUM_FRAMESIZES,&s)<0)break;printf(" %ux%u fps:",s.discrete.width,s.discrete.height);
 for(unsigned k=0;;++k){struct v4l2_frmivalenum v={.index=k,.pixel_format=d.pixelformat,.width=s.discrete.width,.height=s.discrete.height};if(ioctl(f,VIDIOC_ENUM_FRAMEINTERVALS,&v)<0)break;printf(" %.1f",(double)v.discrete.denominator/v.discrete.numerator);}printf("\n");}}
 close(f);return 0;
}
