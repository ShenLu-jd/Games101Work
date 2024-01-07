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

    // 伪代码
    //shade(p, wo)
    //    sampleLight(inter, pdf_light)
    //    Get xx, ws, NN, emit from inter
    //    Shoot a ray from p to xx
    //    If the ray is not blocked in the middle
    //    L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
    //        NN) / |xx - p | ^ 2 / pdf_light
    //    
    //    L_indir = 0.0
    //    Test Russian Roulette with probability RussianRoulette
    //    wi = sample(wo, N)
    //    Trace a ray r(p, wi)
    //    If ray r hit a non - emitting object at q
    //    L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
    //    / pdf(wo, wi, N) / RussianRoulette
    //    
    //    Return L_dir + L_indir


    // 获取视线和物体的交点
    Intersection intersection = Scene::intersect(ray);

    // 没有相交
    if (!intersection.happened) return Vector3f();
    // 打到光源，直接返回光源的颜色
    if (intersection.m->hasEmission()) return intersection.m->getEmission();

    // 光源采样
    Intersection light_pos;
    float pdf_light = 0.0f;
    sampleLight(light_pos, pdf_light);

    Vector3f p, xx, wo, ws, N, NN, emit, ligthToObj;
    p = intersection.coords;    // 与物体的相交点
    xx = light_pos.coords;      // 光源位置
    ligthToObj = (p - xx).normalized();    // 光源 到 物体交点    
    wo = ray.direction;        // 相交点的出射方向
    ws = ligthToObj;            // 相交点与光源的方向
    N = intersection.normal;    // 相交点的法线
    NN = light_pos.normal;      // 相交点与光源的交点的法线
    emit = light_pos.emit;      // 光源强度

    Ray lightToObjRay(xx, ligthToObj);
    // 获取 光源 到 物体交点 的交点
    Intersection lightToScene = Scene::intersect(lightToObjRay);

    Vector3f L_dir = 0.0f, L_indir = 0.0f;  // 直接光照、间接光照
    float dis = (p -xx).norm();
    float dis2 = dotProduct((p - xx), (p - xx));

    // 物体交点和光源之间没有其他物体遮挡
    if (lightToScene.happened && dis - lightToScene.distance < EPSILON)
    {
        // 直接光
        L_dir = emit * intersection.m->eval(wo, -ws, N) * dotProduct(-ws, N) * dotProduct(ws, NN)
            / dis2 / pdf_light;
    }

    // 俄罗斯轮盘赌
    float ksi = get_random_float();
    if (ksi < RussianRoulette)
    {
        // 入射光采样
        Vector3f wi = intersection.m->sample(wo, N).normalized();
        Ray r(p, wi);
        Intersection objToScene = intersect(r);

        if (objToScene.happened && !objToScene.m->hasEmission())
        {
            // 间接光
            L_indir = castRay(r, depth + 1) * intersection.m->eval(wo, wi, N) * dotProduct(wi, N)
                / intersection.m->pdf(wo, wi, N) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}