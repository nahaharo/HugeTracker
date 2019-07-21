#include "pch.h"

#include <Windows.h>
#include "hugetracker.h"

int main()
{
	std::string url = "rtspsrc location=rtsp://admin:kaya1202ha@192.168.219.110:554/Stream/Channels/102  latency=30 ! decodebin ! videoconvert ! appsink";
	//std::string url = "filesrc location=test.mp4 ! decodebin ! videoconvert ! appsink";
	auto t = huge::HugeTracker::GetInstance(url);

	while (true)
	{
		//cpp17 standard: structural binding
		auto[a, b] = t->get_transform(0);
		std::cout << "x: " << a[0] << " y: " << a[1] << " z: " << a[2] << std::endl;
		Sleep(100);
	}
}

