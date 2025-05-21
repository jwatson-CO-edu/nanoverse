import pymesh

A = pymesh.load("pymesh.ply")
B = pymesh.load_mesh("plate.ply")
intersection = pymesh.boolean(A, B, "intersection")