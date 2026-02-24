#include "make_unique.h"
#include "world.h"
#include "quad-tree.h"
#include <algorithm>
#include <iostream>

// TASK 1

// NOTE: You may modify any of the contents of this file, but preserve all function types and names.
// You may add new functions if you believe they will be helpful.

const int QuadTreeLeafSize = 8;
class SequentialNBodySimulator : public INBodySimulator
{
public:
    std::shared_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> & particles, Vec2 bmin, Vec2 bmax)
    {
        auto node = std::make_shared<QuadTreeNode>();
        Vec2 extent = bmax - bmin;
        const float minExtent = 1e-5f;

        if (particles.size() <= QuadTreeLeafSize ||
            (extent.x <= minExtent && extent.y <= minExtent))
        {
            node->isLeaf = true;
            node->particles = particles;
            return node;
        }

        node->isLeaf = false;
        Vec2 pivot = (bmin + bmax) * 0.5f;
        std::vector<Particle> childParticles[4];
        for (auto & p : particles)
        {
            int childIndex = (p.position.x < pivot.x ? 0 : 1) +
                             ((p.position.y < pivot.y ? 0 : 1) << 1);
            childParticles[childIndex].push_back(p);
        }

        Vec2 halfSize = extent * 0.5f;
        for (int i = 0; i < 4; i++)
        {
            Vec2 childBMin;
            childBMin.x = (i & 1) ? pivot.x : bmin.x;
            childBMin.y = ((i >> 1) & 1) ? pivot.y : bmin.y;
            Vec2 childBMax = childBMin + halfSize;
            node->children[i] = buildQuadTree(childParticles[i], childBMin, childBMax);
        }
        return node;
    }
    virtual std::shared_ptr<AccelerationStructure> buildAccelerationStructure(std::vector<Particle> & particles)
    {
        // build quad-tree
        auto quadTree = std::make_shared<QuadTree>();

        // find bounds
        Vec2 bmin(1e30f, 1e30f);
        Vec2 bmax(-1e30f, -1e30f);

        for (auto &p : particles)
        {
            bmin.x = fminf(bmin.x, p.position.x);
            bmin.y = fminf(bmin.y, p.position.y);
            bmax.x = fmaxf(bmax.x, p.position.x);
            bmax.y = fmaxf(bmax.y, p.position.y);
        }

        quadTree->bmin = bmin;
        quadTree->bmax = bmax;

        // build nodes
        quadTree->root = buildQuadTree(particles, bmin, bmax);
        if (!quadTree->checkTree()) {
          std::cout << "Your Tree has Error!" << std::endl;
        }

        return quadTree;
    }
    virtual void simulateStep(AccelerationStructure * accel, std::vector<Particle> & particles, std::vector<Particle> & newParticles, StepParameters params) override
    {
        auto quadTree = static_cast<QuadTree*>(accel);
        if (!quadTree)
        {
            return;
        }

        std::vector<Particle> nearbyParticles;
        for (int i = 0; i < (int)particles.size(); i++)
        {
            const Particle & pi = particles[i];
            nearbyParticles.clear();
            quadTree->getParticles(nearbyParticles, pi.position, params.cullRadius);

            Vec2 force(0.0f, 0.0f);
            for (auto & pj : nearbyParticles)
            {
                if (pj.id == pi.id)
                    continue;
                force += computeForce(pi, pj, params.cullRadius);
            }
            newParticles[i] = updateParticle(pi, force, params.deltaTime);
        }
    }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator()
{
    return std::make_unique<SequentialNBodySimulator>();
}
