//Functions written by Virgile Uytterhaegen
//Structure of a pixel, it has x and y coordinates, R,G,B values and a type, obstacle or empty
typedef struct pixel {
	int y;
	int x;
	int R;
	int G;
	int B;
	int type;
} Pixel;

//Set the initial coordinates according to the type (small/big) of map and for the big one the starting area (right/left)
void reset_coord(int map_type, int area);// Made by Hugo

//First version of the update of the coordinates when considering only trajectories to N,S,E,W
void update_coord(double diff, int angle);// Made by Hugo

//Second version that takes into account angles different from N,S,E,W
void update_coord_compass(double diff, double angle); // Made by Hugo

//Update a specific pixel on the map when an obstacle has been encountered
void draw_map(int m, int n, Pixel map[m][n],int i, int j, int type);

//Draw the borders of the small map
void initialize_small_map(int m, int n, Pixel map[m][n]);

//Display the map on the terminal
void print_map(int m, int n, Pixel map[m][n], int map_size);

//Used in print map to print a pixel
void print_pixel(Pixel pixel);


