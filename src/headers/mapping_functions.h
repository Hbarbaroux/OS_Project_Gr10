typedef struct pixel {
	int y;
	int x;
	int R;
	int G;
	int B;
	int type;
} Pixel;

void reset_coord(int map_type, int area);

void update_coord(double diff, int angle);

void update_coord_compass(double diff, double angle);

void draw_map(int m, int n, Pixel map[m][n],int i, int j, int type);

void initialize_small_map(int m, int n, Pixel map[m][n]);

void print_map(int m, int n, Pixel map[m][n], int map_size);

void print_pixel(Pixel pixel);


