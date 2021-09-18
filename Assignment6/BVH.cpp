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

    if (splitMethod == SplitMethod::NAIVE)
    {
        root = recursiveBuild(primitives);
    }
    else if(splitMethod == SplitMethod::SAH)
    {
        root = recursiveSAHBuild(primitives);
    }

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
        int dim = centroidBounds.maxExtent(); // 0 x轴更长，1 y轴更长，2 z轴更长，下面按照最长边的轴对objects进行排序，再分成两份
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

float BoundsArea(Bounds3 bound)
{
    Vector3f diagonal = bound.Diagonal();
    float area = 2 * (diagonal.x * diagonal.y + diagonal.x * diagonal.z + diagonal.y * diagonal.z);
    return area;
}

float CalculateCost(Bounds3 bounds, std::vector<Object*> objects)
{
    float objectsArea = 0.0f;
    for (int i = 0; i < objects.size(); i++)
    {
        objectsArea += BoundsArea(objects[i]->getBounds());
    }
    return objects.size() * objectsArea / BoundsArea(bounds);
}

BVHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    if (objects.empty())
        return node;

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
        node->left = recursiveSAHBuild(std::vector{ objects[0] });
        node->right = recursiveSAHBuild(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 totalBounds, centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            totalBounds = Union(centroidBounds, objects[i]->getBounds());
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        int bucketCount = 8;

        std::vector<Object*> objectsA = {};
        std::vector<Object*> objectsB = {};
        float minCost = std::numeric_limits<float>::max();

        int dim = centroidBounds.maxExtent();
        float dimExtent = centroidBounds.Diagonal()[dim] / bucketCount;

        for (int i = 1; i < bucketCount; i++)
        {
            std::vector<Object*> tempObjectsA = {};
            std::vector<Object*> tempObjectsB = {};
            Bounds3 tempBoundsA, tempBoundsB;
            float splitLine = centroidBounds.pMin[dim] + i * dimExtent;
            for (int j = 0; j < objects.size(); j++)
            {
                Bounds3 bounds = objects[j]->getBounds();
                Vector3f centroid = bounds.Centroid();
                if (centroid[dim] <= splitLine)
                {
                    tempObjectsA.push_back(objects[j]);
                    tempBoundsA = Union(tempBoundsA, bounds);
                }
                else
                {
                    tempObjectsB.push_back(objects[j]);
                    tempBoundsB = Union(tempBoundsB, bounds);;
                }
            }
            if (tempObjectsA.empty() || tempObjectsB.empty())
            {
                continue;
            }
            float cost = CalculateCost(tempBoundsA, tempObjectsA) + CalculateCost(tempBoundsB, tempObjectsB);
            if (cost < minCost)
            {
                minCost = cost;
                objectsA.clear();
                objectsB.clear();
                objectsA.swap(tempObjectsA);
                objectsB.swap(tempObjectsB);
                //objectsA.assign(tempObjectsA.begin(), tempObjectsA.end()); // 两种写法均可以
                //objectsB.assign(tempObjectsB.begin(), tempObjectsB.end());
            }
        }
        
        node->left = recursiveSAHBuild(objectsA);
        node->right = recursiveSAHBuild(objectsB);
        node->bounds = totalBounds;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getBVHIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getBVHIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    Intersection result;
    std::array<int, 3> DirIsNeg = { (int)(ray.direction.x > 0), (int)(ray.direction.y > 0), (int)(ray.direction.z > 0) };
    if (!node->bounds.IntersectP(ray, ray.direction_inv, DirIsNeg)) // 需要先判断bvh整体是否与光线有交点，避免再去与子节点计算交点。这样会提高效率
        return result;
    if (node->object)
    {
        result = node->object->getObjectIntersection(ray);
    }
    else
    {
        Intersection hit1;
        Intersection hit2;
        if (node->right != nullptr)
        {
            hit1 = getBVHIntersection(node->right, ray);
        }
        if (node->left != nullptr)
        {
            hit2 = getBVHIntersection(node->left, ray);
        }
        result = hit1.distance < hit2.distance ? hit1 : hit2;
    }
    return result;
}