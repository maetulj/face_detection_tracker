/**
 * Storage class for detected faces.
 *
 *
 */

#ifndef FACES_STORAGE_H
#define FACES_STORAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class FacesStorage
{
public:
    int id;

    cv::Point m1;
    cv::Point m2;

    int width;
    int height;

    struct
    {
        int x;
        int y;
    } center;

    int trackedPersonId;

    int trackedPerson;

    /**
     * Constructor.
     */
    FacesStorage()
    {}

    /**
     * Constructor.
     */
    FacesStorage(int id, cv::Point m1, cv::Point m2, int width, int height)
     : id(id)
     , m1(m1)
     , m2(m2)
     , width(width)
     , height(height)
    {
        // Calculate the center.
        center.x = m1.x + 0.5 * width;
        center.y = m2.y + 0.5 * height;
    }

    /**
     * Destructor.
     */
    ~FacesStorage()
    {}
};

#endif // FACES_STORAGE_H
