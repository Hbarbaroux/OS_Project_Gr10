struct pixel {
        int x;
        int y;
        int R;
        int G;
        int B;
};

void reset_coord(void);

void update_coord(double diff, int angle);

void draw_map(int i, int j, int type);


