#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

struct boundingBox {
    std::string classOfObject;
    double probability;
    long xmin;
    long ymin;
    long xmax;
    long ymax;
};

#endif // BOUNDINGBOX_HPP
