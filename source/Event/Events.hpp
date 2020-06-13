#pragma once
#ifndef EVENTS_HPP
#define EVENTS_HPP
namespace Mona
{
	struct Event {

	};

	struct WindowResizeEvent : Event {
		int width;
		int height;
	};

	struct MouseScrollEvent : Event {
		double xOffset;
		double yOffset;
	};
}
#endif