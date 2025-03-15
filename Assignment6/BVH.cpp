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
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }

    if (objects.size() == 1) //�����������������ֻ��һ�����壬��Ϊ�ӽڵ�
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) //���������ֻʣ�������壬ֱ�ӷ���Ϊ���Ҳ��ɻ��ֵ�����
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;//ȡ�����µ����ж��󼸺����ĵ����һ��AABB�߽�
        for (int i = 0; i < objects.size(); ++i)
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }
        
        int dimIndex = centroidBounds.maxExtentDimensionIndex();
        switch (dimIndex) //���򳡾�������
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
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
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    Intersection isect;
    isect.happened = false;

    if (!node->bounds.IntersectP(ray))//���Χ���޽�
    {
        return isect;
    }

    //�ж���Χ���ڸ�BVHBuildNode�������ཻ���
    //����Ҷ�ӽڵ㣬����ģ��ϸ���ж�
    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);//��ʵ����
    }
    //����������������Χ�м�����
    Intersection hitLeft = getIntersection(node->left, ray);
#ifdef RECORD_RAY_HIT_PATH
    if (node->left)
    {
        node->left->rayHitNodePathParent = node;
    }
#endif

    Intersection hitRight = getIntersection(node->right, ray);
#ifdef RECORD_RAY_HIT_PATH
    if (node->right)
    {
        node->right->rayHitNodePathParent = node;
    }
#endif
    
#ifdef RECORD_RAY_HIT_PATH
    //jingz ����ֵ
    if (hitLeft.distance < hitRight.distance)
    {
        node->rayHitNodePathRight = nullptr;

        if (hitLeft.happened)
        {
            node->rayHitNodePathLeft = node->left;
            node->left->rayHitNodePathParent = node;
            hitLeft.curBVHNode = node;
        }
        else
        {
            node->rayHitNodePathLeft = nullptr;
            hitLeft.curBVHNode = nullptr;
        }

        return hitLeft;
    }
    else
    {
        node->rayHitNodePathLeft = nullptr;

        if (hitRight.happened)
        {
            node->rayHitNodePathRight = node->right;
            node->right->rayHitNodePathParent = node;
            hitRight.curBVHNode = node;
        }
        else
        {
            node->rayHitNodePathLeft = nullptr;
            hitRight.curBVHNode = nullptr;
        }

        return hitRight;
    }
#else
    return hitLeft.distance < hitRight.distance ? hitLeft : hitRight;
#endif
}