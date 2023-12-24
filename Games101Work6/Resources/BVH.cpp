#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else if (splitMethod == SplitMethod::SAH)
        root = recursiveBuildSAH(primitives);

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

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}


// 使用SAH算法进行空间划分，优化划分的方法，需要多个模型测试才有效果，只有一个模型没有效果 
// 参考 https://zhuanlan.zhihu.com/p/477316706
BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
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
        node->left = recursiveBuildSAH(std::vector{ objects[0] });
        node->right = recursiveBuildSAH(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        auto beginning = objects.begin();
        auto ending = objects.end();

        // 认为根节点物体数量小于12时 对半分的效率更高
        if (objects.size() < 12)
        {
            auto middling = beginning + objects.size() / 2;
            auto leftShapes = std::vector<Object*>(beginning, middling);
            auto rightShapes = std::vector<Object*>(middling + 1, ending);
            node->left = recursiveBuildSAH(leftShapes);
            node->right = recursiveBuildSAH(rightShapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else
        {
            int bestChoice = 0;
            double minCost = std::numeric_limits<double >::max();
            int bestDim = 0;

            for (int dim = 0; dim < 3; dim++)
            {
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

                float l = (float)objects.size();
                // 手动划分为6格
                float nums[] = { 1.0 / 6, 2.0 / 6, 3.0 / 6, 4.0 / 6, 5.0 / 6 };
                for (int i = 0; i < 5; i++)
                    nums[i] *= l;

                for (int i = 0; i < 5; i++)
                {
                    auto middling = objects.begin() + (int)nums[i];
                    auto leftShapes = std::vector<Object*>(beginning, middling);
                    auto righShapes = std::vector<Object*>(middling + 1, ending);
                    auto leftBound = computeSize(leftShapes);
                    auto rightBound = computeSize(righShapes);
                    double cost = 100.0f + (leftBound.SurfaceArea() + rightBound.SurfaceArea()) / bounds.SurfaceArea();

                    if (cost < minCost)
                    {
                        minCost = cost;
                        bestChoice = i;
                        bestDim = dim;
                    }
                }
            }

            if (bestDim == 0)
            {
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                    });
            }
            else if (bestDim == 1)
            {
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                    });
            }


            auto beginning = objects.begin();
            auto middling = objects.begin() + bestChoice;
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuildSAH(leftshapes);
            node->right = recursiveBuildSAH(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }
    return node;
}

Bounds3 BVHAccel::computeSize(std::vector<Object*> objects)
{
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    return bounds;
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
    // 没有交点
    if (!node->bounds.IntersectP(ray, ray.direction_inv)) return Intersection();

    // 叶子节点且有交点
    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);
    }

    // 二叉树继续递归
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);
    
    // 获取离摄像机近的点
    return hit1.distance < hit2.distance ? hit1 : hit2;
}