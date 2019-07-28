#include "pch.h"

#include <Windows.h>
#include "hugetracker.h"

int main()
{
	//std::string url = "rtspsrc location=rtsp://admin:kaya1202ha@192.168.219.110:554/Stream/Channels/102  latency=30 ! decodebin ! videoconvert ! appsink";
	std::string url = "filesrc location=video4.mp4 ! decodebin ! videoconvert ! appsink";
	auto t = huge::HugeTracker::GetInstance(url);

	while (true)
	{
		//cpp17 standard: structural binding
		auto[a, b] = t->get_transform(0);
		std::cout << "x: " << b[0] << " y: " << b[1] << " z: " << b[2] << std::endl;
		Sleep(100);
	}
}

