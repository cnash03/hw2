#include "make_unique.h"
#include "world.h"
#include <iostream>

class SimpleNBodySimulator : public INBodySimulator
{
public:
    std::vector<Particle> nearbyParticles;

    virtual std::shared_ptr<AccelerationStructure> buildAccelerationStructure(std::vector<Particle> & /*particles*/) override
    {
        // don't build any acceleration structures
        return nullptr;
    }

    virtual void simulateStep(AccelerationStructure * accel, std::vector<Particle> & particles, std::vector<Particle> & newParticles, StepParameters params) override
{
    static int stepCount = 0;
    stepCount++;

    #pragma omp parallel for
    for (int i = 0; i < (int)particles.size(); i++)
    {
        auto pi = particles[i];
        Vec2 force = Vec2(0.0f, 0.0f);
        int neighborCount = 0;
        for (size_t j = 0; j < particles.size(); j++)
        {
            if (j == i) continue;
            if ((pi.position - particles[j].position).length() < params.cullRadius)
            {
                force += computeForce(pi, particles[j], params.cullRadius);
                neighborCount++;
            }
        }
        newParticles[i] = updateParticle(pi, force, params.deltaTime);

        // DEBUG: dump particle 444 after step 1
        if (stepCount == 1 && i == 443)
        {
            fprintf(stderr, "[SIMPLE STEP 1] particle 444:\n");
            fprintf(stderr, "  pos=(%.9f, %.9f)\n", newParticles[i].position.x, newParticles[i].position.y);
            fprintf(stderr, "  vel=(%.9f, %.9f)\n", newParticles[i].velocity.x, newParticles[i].velocity.y);
            fprintf(stderr, "  force=(%.9f, %.9f)\n", force.x, force.y);
            fprintf(stderr, "  num neighbors: %d\n", neighborCount);
        }
    }
}
};

std::unique_ptr<INBodySimulator> createSimpleNBodySimulator()
{
    return std::make_unique<SimpleNBodySimulator>();
}
