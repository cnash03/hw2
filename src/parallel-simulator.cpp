#include "make_unique.h"
#include "world.h"
#include "quad-tree.h"
#include <algorithm>
#include <iostream>

// TASK 2

// NOTE: You may modify this class definition as you see fit, as long as the class name,
// and type of simulateStep and buildAccelerationStructure remain the same.

const int QuadTreeLeafSize = 8;
class ParallelNBodySimulator : public INBodySimulator
{
public:
    // TODO: implement a function that builds and returns a quadtree containing particles.
    // You do not have to preserve this function type.
    std::shared_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> & particles, Vec2 bmin, Vec2 bmax, int depth)
    {
        auto node = std::make_shared<QuadTreeNode>();

        if (particles.size() <= QuadTreeLeafSize){
            node->isLeaf = true;
            node->particles = particles;
            return node;
        }
        else{
            node->isLeaf = false;
            auto mid = (bmin + bmax) * 0.5f;
            std::vector<Particle> q0, q1, q2, q3;
            for (auto p : particles){
                if (p.position.y < mid.y)
                {
                    if (p.position.x < mid.x) q0.push_back(p);
                    else q1.push_back(p);
                }
                else
                {
                    if (p.position.x < mid.x) q2.push_back(p);
                    else q3.push_back(p);
                }
            }

            if (depth < 4)
            {
                #pragma omp task shared(node)
                    node->children[0] = buildQuadTree(q0, bmin, mid, depth + 1);
                #pragma omp task shared(node)
                    node->children[1] = buildQuadTree(q1, Vec2(mid.x, bmin.y), Vec2(bmax.x, mid.y), depth + 1);
                #pragma omp task shared(node)
                    node->children[2] = buildQuadTree(q2, Vec2(bmin.x, mid.y), Vec2(mid.x, bmax.y), depth + 1);
                #pragma omp task shared(node)
                    node->children[3] = buildQuadTree(q3, mid, bmax, depth + 1);
                #pragma omp taskwait
            }
            else
            {
                node->children[0] = buildQuadTree(q0, bmin, mid, depth + 1);
                node->children[1] = buildQuadTree(q1, Vec2(mid.x, bmin.y), Vec2(bmax.x, mid.y), depth + 1);
                node->children[2] = buildQuadTree(q2, Vec2(bmin.x, mid.y), Vec2(mid.x, bmax.y), depth + 1);
                node->children[3] = buildQuadTree(q3, mid, bmax, depth + 1);
            }
        }
        return node;
    }

    // Do not modify this function type.
    virtual std::shared_ptr<AccelerationStructure> buildAccelerationStructure(std::vector<Particle> & particles)
    {
        // build quad-tree
        auto quadTree = std::make_shared<QuadTree>();

        // find bounds
        Vec2 bmin(1e30f, 1e30f);
        Vec2 bmax(-1e30f, -1e30f);

        for (auto & p : particles)
        {
            bmin.x = fminf(bmin.x, p.position.x);
            bmin.y = fminf(bmin.y, p.position.y);
            bmax.x = fmaxf(bmax.x, p.position.x);
            bmax.y = fmaxf(bmax.y, p.position.y);
        }

        quadTree->bmin = bmin;
        quadTree->bmax = bmax;

        // build nodes
        quadTree->root = buildQuadTree(particles, bmin, bmax, 0);
        if (!quadTree->checkTree()) {
          std::cout << "Your Tree has Error!" << std::endl;
        }

        return quadTree;
    }

    // Do not modify this function type.
    virtual void simulateStep(AccelerationStructure * accel, std::vector<Particle> & particles, std::vector<Particle> & newParticles, StepParameters params) override
    {
        #pragma omp parallel for schedule(dynamic,64)
        for (int i = 0; i < (int)particles.size(); i++)
        {
            auto &pi = particles[i];

            std::vector<Particle> nearbyParticles;
            accel->getParticles(nearbyParticles, pi.position, params.cullRadius);

            Vec2 force(0.0f, 0.0f);

            for (auto &pj : nearbyParticles)
            {
                if (pj.id == pi.id) continue;
                force += computeForce(pi, pj, params.cullRadius);
            }

            newParticles[i] = updateParticle( pi, force, params.deltaTime);
        }
    }
};

// Do not modify this function type.
std::unique_ptr<INBodySimulator> createParallelNBodySimulator()
{
  return std::make_unique<ParallelNBodySimulator>();
}







