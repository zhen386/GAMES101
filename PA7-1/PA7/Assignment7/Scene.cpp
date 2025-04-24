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
        float &tNear, uint32_t &indelight_point, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indelight_pointK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indelight_pointK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            indelight_point = indelight_pointK;
        }
    }


    return (*hitObject != nullptr);
}

// Vector3f Scene::castRay(const Ray& ray, int depth) const
// {
//     // TO DO Implement Path Tracing Algorithm here
//     Vector3f hitColor(0);
//     Intersection shade_point_inter = Scene::intersect(ray);
//     if (shade_point_inter.happened)
//     {
//
//         Vector3f p = shade_point_inter.coords;
//         Vector3f wo = ray.direction;
//         Vector3f N = shade_point_inter.normal;
//         Vector3f L_dir(0), L_indir(0);
//
//         //sampleLight(inter,pdf_light)
//         Intersection light_point_inter;
//         float pdf_light;
//         sampleLight(light_point_inter, pdf_light);
//         //Get light_point,ws,NN,emit from inter
//         Vector3f light_point = light_point_inter.coords;
//         Vector3f ws = normalize(light_point-p);
//         Vector3f NN = light_point_inter.normal;
//         Vector3f emit = light_point_inter.emit;
//         float distance_pTolight_point = (light_point - p).norm();
//         //Shoot a ray from p to light_point
//         Vector3f p_deviation = (dotProduct(ray.direction, N) < 0) ?
//                 p + N * EPSILON :
//                 p - N * EPSILON ;
//         Ray ray_pTolight_point(p_deviation, ws);
//         //If the ray is not blocked in the middleff
//         Intersection blocked_point_inter = Scene::intersect(ray_pTolight_point);
//         if (abs(distance_pTolight_point - blocked_point_inter.distance < 0.01 ))
//         {
//             L_dir = emit * shade_point_inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (distance_pTolight_point * distance_pTolight_point * pdf_light);
//         }
//         //Test Russian Roulette with probability RussianRouolette
//         float ksi = get_random_float();
//         if (ksi < RussianRoulette)
//         {
//             //wi=sample(wo,N)
//             Vector3f wi = normalize(shade_point_inter.m->sample(wo, N));
//             //Trace a ray r(p,wi)
//             Ray ray_pTowi(p_deviation, wi);
//             //If ray r hit a non-emitting object at q
//             Intersection bounce_point_inter = Scene::intersect(ray_pTowi);
//             if (bounce_point_inter.happened && !bounce_point_inter.m->hasEmission())
//             {
//                 float pdf = shade_point_inter.m->pdf(wo, wi, N);
//                 if(pdf> EPSILON)
//                     L_indir = castRay(ray_pTowi, depth + 1) * shade_point_inter.m->eval(wo, wi, N) * dotProduct(wi, N) / (pdf *RussianRoulette);
//             }
//         }
//         hitColor = shade_point_inter.m->getEmission() + L_dir + L_indir;
//     }
//     return hitColor;
// }

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO:Implement Path Tracing Algorithm here

    Intersection shade_point_inter = intersect(ray);
    if (!shade_point_inter.happened) return this->backgroundColor;
    if (shade_point_inter.m->hasEmission()) return shade_point_inter.m->getEmission();

    Intersection light_point_inter;
    float pdf_light;
    sampleLight(light_point_inter, pdf_light);

    Vector3f light_point = light_point_inter.coords;
    Vector3f shade_point = shade_point_inter.coords;
    Vector3f light_to_shade_direction = (light_point - shade_point).normalized();
    Vector3f light_emit = light_point_inter.emit;
    Vector3f shade_point_normal = shade_point_inter.normal.normalized();
    Vector3f light_point_normal = light_point_inter.normal.normalized();

    Vector3f L_direct = 0;;
    Vector3f L_indirect = 0;;

    Ray light_to_shade_ray(shade_point, light_to_shade_direction);
    Intersection light_to_shade_inter = intersect(light_to_shade_ray);
    float distance = (light_point - shade_point).norm();
    float light_cosine = dotProduct(-light_to_shade_ray.direction, light_point_normal);

    if (light_to_shade_inter.distance - distance > -0.01 && pdf_light > EPSILON && light_cosine > EPSILON) {
        L_direct = light_emit
                * shade_point_inter.m->eval(ray.direction, light_to_shade_ray.direction, shade_point_normal)
                * dotProduct(light_to_shade_ray.direction, shade_point_normal)
                * light_cosine / std::pow(distance, 2) / pdf_light;
    }

    if(get_random_float() > RussianRoulette) return L_direct;

    Vector3f bounce_direction = shade_point_inter.m->sample(ray.direction, shade_point_normal).normalized();
    Ray bounce_to_shade_ray(shade_point, bounce_direction);
    Intersection bounce_to_shade_inter = intersect(bounce_to_shade_ray);


    if (bounce_to_shade_inter.happened && !(bounce_to_shade_inter.m->hasEmission())) {
        float pdf = shade_point_inter.m->pdf(ray.direction, bounce_to_shade_ray.direction, shade_point_normal);
        if (pdf > EPSILON) {
            L_indirect = castRay(bounce_to_shade_ray, depth + 1)
                    * shade_point_inter.m->eval(ray.direction, bounce_to_shade_ray.direction, shade_point_normal)
                    * dotProduct(bounce_to_shade_ray.direction, shade_point_normal)
                    / pdf / RussianRoulette;
        }
    }

    return L_direct + L_indirect;
}

