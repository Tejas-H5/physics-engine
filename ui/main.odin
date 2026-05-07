package ui

Rect :: struct {
	x, y, w, h: f32
}

rect_from_size :: proc(width, height: f32) -> Rect {
	return { w=width, h=height }
}

// BUT. how do I make a rectangle that automatically sizes to it's contents? Can't do it.
// the UI_PHASES is the only way. but I dont want to constantly measure and then draw all my UI...
// arg.

inset :: proc(rect: ^Rect, amount: f32) {
	cut_left(rect, amount)
	cut_right(rect, amount)
	cut_top(rect, amount)
	cut_bottom(rect, amount)
}

cut_left :: proc(rect: ^Rect, amount: f32) -> (Rect, bool) #optional_ok {
	if amount > rect.w {
		if rect.w > 0 { rect.w -= amount }
		return {}, false
	}

	result := rect^
	result.x = rect.x
	result.w = amount
	
	rect.x += amount
	rect.w -= amount

	return result, true
}

cut_right :: proc(rect: ^Rect, amount: f32) -> (Rect, bool) #optional_ok {
	if amount > rect.w {
		if rect.w > 0 { rect.w -= amount }
		return {}, false
	}

	result := rect^
	result.x = rect.x + rect.w - amount
	result.w = amount
	
	rect.w -= amount

	return result, true
}

cut_top :: proc(rect: ^Rect, amount: f32) -> (Rect, bool) #optional_ok {
	if amount > rect.h {
		if rect.h > 0 { rect.h -= amount }
		return {}, false
	}

	result := rect^
	result.y = rect.y
	result.h = amount
	
	rect.y += amount
	rect.h -= amount

	return result, true
}

cut_bottom :: proc(rect: ^Rect, amount: f32) -> (Rect, bool) #optional_ok {
	if amount > rect.h {
		if rect.h > 0 { rect.h -= amount }
		return {}, false
	}

	result := rect^
	result.y = rect.y + rect.h - amount
	result.h = amount
	
	rect.h -= amount

	return result, true
}

