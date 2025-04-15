#include <algorithm>
#include <cassert>
#include "BVH.hpp"

float getDimValue(int dim, Vector3f v) {
    if (dim == 0) return v.x;
    else if (dim == 1) return v.y;
    else return v.z;
}

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        if (splitMethod == SplitMethod::NAIVE) {

            //std::cout << "NAIVE!" << std::endl;

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        } else {
            int n = 10;
            int minSplit = 0;

            switch (dim) {
                case 0: {
                    float split_size = (bounds.pMax.x - bounds.pMin.x) / n;
                    float split = bounds.pMin.x + split_size;

                    for (int i = 0; i < n; ++i) {
                        Bounds3 leftBounds, rightBounds;
                        int leftCount = 0, rightCount = 0;
                        float minCost = std::numeric_limits<float>::max();

                        for (Object* object : objects) {

                            if (object->getBounds().Centroid().x <= split) {
                                leftCount ++;
                                leftBounds = Union(leftBounds, object->getBounds());
                            } else {
                                rightCount ++;
                                rightBounds = Union(rightBounds, object->getBounds());
                            }
                        }

                        float cost = 0.125 + leftBounds.SurfaceArea() / bounds.SurfaceArea() * leftCount
                                        + rightBounds.SurfaceArea() / bounds.SurfaceArea() * rightCount;
                        if (cost < minCost) {
                            minCost = cost;
                            minSplit = leftCount;
                        }
                        split += split_size;
                    }
                }

                case 1: {

                    float split_size = (bounds.pMax.y - bounds.pMin.y) / n;
                    float split = bounds.pMin.y + split_size;

                    for (int i = 0; i < n; ++i) {
                        Bounds3 leftBounds, rightBounds;
                        int leftCount = 0, rightCount = 0;
                        float minCost = std::numeric_limits<float>::max();

                        for (Object* object : objects) {

                            if (object->getBounds().Centroid().y <= split) {
                                leftCount ++;
                                leftBounds = Union(leftBounds, object->getBounds());
                            } else {
                                rightCount ++;
                                rightBounds = Union(rightBounds, object->getBounds());
                            }
                        }

                        float cost = 0.125 + leftBounds.SurfaceArea() / bounds.SurfaceArea() * leftCount
                                        + rightBounds.SurfaceArea() / bounds.SurfaceArea() * rightCount;
                        if (cost < minCost) {
                            minCost = cost;
                            minSplit = leftCount;
                        }
                        split += split_size;
                    }
                }

                case 2: {

                    float split_size = (bounds.pMax.z - bounds.pMin.z) / n;
                    float split = bounds.pMin.z + split_size;

                    for (int i = 0; i < n; ++i) {
                        Bounds3 leftBounds, rightBounds;
                        int leftCount = 0, rightCount = 0;
                        float minCost = std::numeric_limits<float>::max();

                        for (Object* object : objects) {

                            if (object->getBounds().Centroid().z <= split) {
                                leftCount ++;
                                leftBounds = Union(leftBounds, object->getBounds());
                            } else {
                                rightCount ++;
                                rightBounds = Union(rightBounds, object->getBounds());
                            }
                        }

                        float cost = 0.125 + leftBounds.SurfaceArea() / bounds.SurfaceArea() * leftCount
                                        + rightBounds.SurfaceArea() / bounds.SurfaceArea() * rightCount;
                        if (cost < minCost) {
                            minCost = cost;
                            minSplit = leftCount;
                        }
                        split += split_size;
                    }
                }
            }



            // float bounds_pMax = getDimValue(dim, centroidBounds.pMax);
            // float bounds_pMin = getDimValue(dim, centroidBounds.pMin);
            //
            // float bucket_num = 10.0;
            // float bucket_size = (bounds_pMax - bounds_pMin) / bucket_num;
            // float bucket_pMax = bounds_pMin + bucket_size;
            //
            // float min_cost = std::numeric_limits<float>::infinity();
            // float minSplit = 0;
            //
            // for(int j = 0; j < bucket_num; j++) {
            //     Bounds3 A, B;
            //     int num_a = 0, num_b = 0;
            //     // find bounding boxes for current split
            //     for (int i = 0; i < objects.size(); ++i) {
            //         float object_centroid = getDimValue(dim, objects[i]->getBounds().Centroid());
            //         if (object_centroid <= bucket_pMax) {
            //             A = Union(A, objects[i]->getBounds());
            //             num_a++;
            //         } else {
            //             B = Union(B, objects[i]->getBounds());
            //             num_b++;
            //         }
            //     }
            //     bucket_pMax += bucket_size;
            //     // find probabilities
            //     float p_a = A.SurfaceArea() / centroidBounds.SurfaceArea();
            //     float p_b = B.SurfaceArea() / centroidBounds.SurfaceArea();
            //     // find cost
            //     float cost = p_a*num_a + p_b*num_b;
            //     if (cost < min_cost) {
            //         min_cost = cost;
            //         minSplit = num_a;
            //     }
            // }

auto beginning = objects.begin();
auto middling = objects.begin() + minSplit;
auto ending = objects.end();

auto leftshapes = std::vector<Object*>(beginning, middling);
auto rightshapes = std::vector<Object*>(middling, ending);

assert(objects.size() == (leftshapes.size() + rightshapes.size()));

node->left = recursiveBuild(leftshapes);
node->right = recursiveBuild(rightshapes);

node->bounds = Union(node->left->bounds, node->right->bounds);

        }
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    //Vector3f invDir(ray.direction_inv)
    Intersection intersection;

    if (!node) return intersection;

    const std::array<int, 3>& dirIsNeg = {(int)(ray.direction.x > 0), (int)(ray.direction.y > 0), (int)(ray.direction.z > 0)};
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) return intersection;

    if (node->object != nullptr) {
        return node->object->getIntersection(ray);
    };

    Intersection hit_left = getIntersection(node->left, ray);
    Intersection hit_right = getIntersection(node->right, ray);

    if (hit_left.happened || hit_right.happened) {
        intersection = hit_left.distance < hit_right.distance ? hit_left : hit_right;
    };

    return intersection;
}

// Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
// {
//
//     // TODO Traverse the BVH to find intersection
//
//     Intersection intr;
//     if(!node) return intr;
//
//     const Vector3f& invDir = {1/ray.direction.x, 1/ray.direction.y, 1/ray.direction.z};
//     const std::array<int, 3>& dirIsNeg = {ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0};
//
//     if(!node->bounds.IntersectP(ray, invDir, dirIsNeg)) return intr;
//
//     if (node->object != nullptr) {
//         return node->object->getIntersection(ray);
//     }
//
//     Intersection hit1 = getIntersection(node->left, ray);
//     Intersection hit2 = getIntersection(node->right, ray);
//     if(hit1.happened || hit2.happened) {
//         return hit1.distance <= hit2.distance ? hit1 : hit2;
//     }
//
//     return intr;
// }