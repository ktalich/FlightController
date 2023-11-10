class Helpers
{
public:
    static float mapf(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    static float clampf(float x, float min, float max)
    {
        if (x < min)
            return min;
        if (x > max)
            return max;
        return x;
    }
};
