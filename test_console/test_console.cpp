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
		std::array<float, 3> l, f, u;
		t->get_transform(0, l, f, u);
		std::cout << "x: " << u[0] << " y: " << u[1] << " z: " << u[2] << std::endl;
		Sleep(100);
	}
}

