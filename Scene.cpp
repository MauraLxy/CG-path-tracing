//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // Ray interaction
    Intersection intersection = intersect(ray);
    if (!intersection.happened)
    {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }
    // Emission
    if (intersection.m->hasEmission())
    {
        return intersection.m->getEmission();
    }
    // Direct shading
    Intersection lightPosition;
    float lightPDF;
    sampleLight(lightPosition,lightPDF);
    Vector3f lightDirection = (lightPosition.coords-intersection.coords).normalized();
    Ray lightRay(intersection.coords,lightDirection);
    Intersection lightIntersection = intersect(lightRay);
    Vector3f L_dir = Vector3f(0.0f, 0.0f, 0.0f);
    if (lightIntersection.happened && lightIntersection.m->hasEmission())
    {
        Vector3f albedo = intersection.m->eval(ray.direction,lightDirection,intersection.normal);
        float lightDistance = (lightPosition.coords - intersection.coords).norm();
        L_dir = lightIntersection.m->getEmission() * albedo * std::max(0.f,dotProduct(intersection.normal, lightDirection)) / (lightDistance * lightDistance * lightPDF);
    }
    // Sampling Indirect Shading
    Vector3f L_indir = Vector3f(0.0f, 0.0f, 0.0f);
    if (get_random_float() < RussianRoulette)
    {
        Vector3f outRayDirection = intersection.m->sample(ray.direction,intersection.normal).normalized();
        Ray outRay(intersection.coords,outRayDirection);
        Intersection outRayIntersection = intersect(outRay);
        if (outRayIntersection.happened && !outRayIntersection.obj->hasEmit())
        {
            float outPDF = outRayIntersection.m->pdf(ray.direction,outRay.direction,intersection.normal);
            Vector3f albedo = intersection.m->eval(ray.direction,outRay.direction,intersection.normal);
            L_indir = castRay(outRay, depth+1) * albedo * std::max(0.f, dotProduct(intersection.normal,outRay.direction))/(RussianRoulette * outPDF);
        }
    }
    return L_dir + L_indir;
}
