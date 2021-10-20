//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

float P_RR = 0.7f;
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened)
    {
        return this->backgroundColor;
    }

    Vector3f L_dir = 0, L_indir = 0;

    Object* hitObject = intersection.obj;
    Vector3f hitPoint = intersection.coords;
    Material* m = intersection.m;
    Vector3f N = intersection.normal;
    Vector3f wo = ray.direction;

    if (hitObject->hasEmit())
    {
        return intersection.emit;
    }

    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);
    Vector3f lightPos = inter.coords;
    Vector3f ws = normalize(lightPos - hitPoint);
    Intersection inter1 = intersect(Ray(hitPoint, ws));
    if(inter1.happened && inter1.obj->hasEmit())
    {
        Vector3f NN = inter.normal;
        L_dir = inter.emit * m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dotProduct(lightPos - hitPoint, lightPos - hitPoint) / pdf_light;
    }

    // Russian Roulette(RR)
    if (get_random_float() < P_RR)
    {
        Vector3f wi = m->sample(wo, N);
        Ray ray3 = Ray(hitPoint, wi);
        Intersection inter2 = intersect(ray3);
        if (inter2.happened && !inter2.obj->hasEmit())
        {
            L_indir = castRay(ray3, 0) * m->eval(wo, wi, N) * dotProduct(wi, N) / m->pdf(wo, wi, N) / P_RR;
        }
    }

    return L_dir + L_indir;
}