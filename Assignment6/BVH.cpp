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
    else
    {
        buckets = recursiveBuildSAH(primitives);
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

SAHBuildBucket* GetNearestBucket(std::vector<SAHBuildBucket*> buckets, Object* object, int dim)
{
    SAHBuildBucket* bucket = buckets[0];
    float minDistance = std::numeric_limits<float>::infinity();
    float tempValue = 0.0f;
    for (int i = 0; i < buckets.size(); i++)
    {
        switch (dim)
        {
        case 0:
            tempValue = std::abs(buckets[i]->bounds.Centroid().x - object->getBounds().Centroid().x);
            if (tempValue < minDistance)
            {
                bucket = buckets[i];
                minDistance = tempValue;
            }
            break;
        case 1:
            tempValue = std::abs(buckets[i]->bounds.Centroid().y - object->getBounds().Centroid().y);
            if (tempValue < minDistance)
            {
                bucket = buckets[i];
                minDistance = tempValue;
            }
            break;
        case 2:
            tempValue = std::abs(buckets[i]->bounds.Centroid().z - object->getBounds().Centroid().z);
            if (tempValue < minDistance)
            {
                bucket = buckets[i];
                minDistance = tempValue;
            }
            break;
        }
    }
    return bucket;
}

std::vector<SAHBuildBucket*> CreateBuckets(Bounds3 totalBounds, int size, int dim)
{
    std::vector<SAHBuildBucket*> buckets = {};
    Vector3f diagonal = totalBounds.Diagonal() / size;
    for (int i = 0; i < size; i++)
    {
        SAHBuildBucket* bucket = new SAHBuildBucket();
        bucket->bounds.pMin = totalBounds.pMin;
        bucket->bounds.pMax = totalBounds.pMax;
        switch (dim)
        {
        case 0:
            bucket->bounds.pMin.x = totalBounds.pMin.x + i * diagonal.x;
            bucket->bounds.pMax.x = totalBounds.pMin.x + (i + 1) * diagonal.x;
            break;
        case 1:
            bucket->bounds.pMin.y = totalBounds.pMin.y + i * diagonal.y;
            bucket->bounds.pMax.y = totalBounds.pMin.y + (i + 1) * diagonal.y;
            break;
        case 2:
            bucket->bounds.pMin.z = totalBounds.pMin.z + i * diagonal.z;
            bucket->bounds.pMax.z = totalBounds.pMin.z + (i + 1) * diagonal.z;
            break;
        }
        buckets.push_back(bucket);
    }
    return buckets;
}

std::vector<SAHBuildBucket*> BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    Bounds3 totalBounds;
    for (int i = 0; i < objects.size(); ++i)
        totalBounds = Union(totalBounds, objects[i]->getBounds());
    int dim = totalBounds.maxExtent(); // 0 x轴更长，1 y轴更长，2 z轴更长，下面按照最长边的轴对objects进行排序，再分成两份
    int bucketCount = 32;
    std::vector<SAHBuildBucket*> buckets = CreateBuckets(totalBounds, bucketCount, dim);

    for (int i = 0; i < objects.size(); i++)
    {
        SAHBuildBucket* bucket = GetNearestBucket(buckets, objects[i], dim);
        bucket->bounds = Union(bucket->bounds, objects[i]->getBounds());
        bucket->objects.push_back(objects[i]);
    }
    for (int i = 0; i < buckets.size(); i++)
    {
        if (buckets[i]->objects.size() > 0)
        {
            buckets[i]->root = BVHAccel::recursiveBuild(buckets[i]->objects);
        }
    }
    return buckets;
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

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (this->splitMethod == SplitMethod::NAIVE)
    {
        if (!root)
            return isect;
        isect = BVHAccel::getBVHIntersection(root, ray);
        return isect;
    }
    else
    {
        if (buckets.size() == 0)
            return isect;
        isect = BVHAccel::getSAHIntersection(buckets, ray);
        return isect;
    }
}

Intersection BVHAccel::getSAHIntersection(std::vector<SAHBuildBucket*> buckets, const Ray& ray) const
{
    Intersection result;
    Intersection temp;
    std::array<int, 3> DirIsNeg = { (int)(ray.direction.x > 0), (int)(ray.direction.y > 0), (int)(ray.direction.z > 0) };
    for (int i = 0; i < buckets.size(); i++)
    {
        if (buckets[i]->objects.size() == 0)
            continue;
        //if (!buckets[i]->bounds.IntersectP(ray, ray.direction_inv, DirIsNeg)) // 需要先判断bvh整体是否与光线有交点，避免再去与子节点计算交点。这样会提高效率
        //    return result;
        temp = BVHAccel::getBVHIntersection(buckets[i]->root, ray);
        if (!result.happened)
        {
            result = temp;
        }
        else
        {
            if (temp.distance < result.distance)
            {
                result = temp;
            }
        }
    }
    return result;
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