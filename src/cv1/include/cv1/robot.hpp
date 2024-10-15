class Robot
{
public:
    Robot();
    void move(double x, double y);
    double getX() const;
    double getY() const;

private:
    double x_coordinate_;
    double y_coordinate_;
};