#pragma once

namespace mz
{
enum class EventType
{
	eKeyInput,
	eMouseButton,
	eScroll,
	eResize
};

enum class MouseButton
{
	eLeft,
	eRight,
	eMiddle,
	eUnknown
};

enum class KeyCode
{
	eW,
	eS,
	eA,
	eD,
	eUnknown
};

enum class KeyAction
{
	eDown,
	eUp,
	eRepeat,
	eUnknown
};

enum class MouseAction
{
	eDown,
	eUp,
	eMove,
	eUnknown
};

struct Event
{
	Event(EventType type) :
	    type(type){};
	EventType type;
};

struct ResizeEvent : Event
{
	ResizeEvent() :
	    Event(EventType::eResize){};
};

struct KeyEvent : Event
{
	KeyEvent(KeyCode code, KeyAction action) :
	    Event(EventType::eKeyInput),
	    code(code),
	    action(action)
	{
	}
	KeyCode   code;
	KeyAction action;
};

struct MouseButtonEvent : Event
{
	MouseButtonEvent(MouseButton button, MouseAction action, float pos_x, float pos_y) :
	    Event(EventType::eMouseButton),
	    button(button),
	    action(action),
	    xpos(pos_x),
	    ypos(pos_y)
	{
	}
	MouseButton button;
	MouseAction action;
	float       xpos;
	float       ypos;
};

struct ScrollEvent : Event
{
	ScrollEvent(float x_offset, float y_offset) :
	    Event(EventType::eScroll),
	    x_offset(x_offset),
	    y_offset(y_offset)
	{
	}
	float x_offset;
	float y_offset;
};

}        // namespace mz