# THIS IS HUGETRACKER

HugeTracker is IR marker detection project that can estimate pose of user head to make interactive space.



## HugeTracker

Header only project(well we needs some adjustment for it)



## Requirement

### OpenCV

This library needs opencv.

Recommend to compile on working computer.

### GSTREAM

Strongly recommend to use gstream to reduce latency between rtsp stream and opencv.



### Usage Example : test_console/test_console.cpp

```cpp
#include "pch.h"

#include <Windows.h>
#include "hugetracker.h"

int main()
{
	std::string url = "rtspsrc location=rtsp://<ID>:<password>@192.168.219.110:554/Stream/Channels/102  latency=30 ! decodebin ! videoconvert ! appsink";
    //you can set your own video but when video ends, rotation and translation
    //will not change
    
    //std::string url = "filesrc location=${Location to mp4 file} ! decodebin ! videoconvert ! appsink";
    
	auto t = huge::HugeTracker::GetInstance(url);

	while (true)
	{
		//cpp17 standard: structural binding
		auto[a, b] = t->get_transform(0);
		std::cout << "x: " << a[0] << " y: " << a[1] << " z: " << a[2] << std::endl;
		Sleep(100);
	}
}
```

At that case, a is std::array<double, 3> that has transformation of marker, and b is std::array<double, 9> that has rotation matrix. Elements order is like this:

{a[0,0]. a[0,1], a[0,2], .... , a[2,1], a[2,2], a[2,3]}

