def get_vertex_neighbours(pos):
		n = []
		#Moves allow link a chess king
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			if x2 < 0 or x2 > 25 or y2 < 0 or y2 > 25:
				continue
			n.append((x2, y2))
		return n

print(get_vertex_neighbours([0,0]))        