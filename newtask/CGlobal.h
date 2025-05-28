#pragma once
struct Point {
    double x, y, z;
    int label; // segmentation label
    
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        auto h1 = std::hash<double>{}(p.x);
        auto h2 = std::hash<double>{}(p.y);
        auto h3 = std::hash<double>{}(p.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct GridKey {
    int gx, gy, gz;

    bool operator==(const GridKey& other) const {
        return gx == other.gx && gy == other.gy && gz == other.gz;
    }
};

struct GridHash {
    std::size_t operator()(const GridKey& k) const {
        auto h1 = std::hash<int>{}(k.gx);
        auto h2 = std::hash<int>{}(k.gy);
        auto h3 = std::hash<int>{}(k.gz);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};