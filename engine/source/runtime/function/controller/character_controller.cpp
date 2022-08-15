#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Pilot
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PILOT_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(
                Vector3(0, 0, capsule.m_half_height + capsule.m_radius),
                orientation,
                Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        std::shared_ptr<PhysicsScene> physics_scene =
            g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
        ASSERT(physics_scene);

        std::vector<PhysicsHitInfo> hits;

        Transform world_transform = Transform(
            current_position + 0.1f * Vector3::UNIT_Z,
            Quaternion::IDENTITY,
            Vector3::UNIT_SCALE);

        Vector3 vertical_displacement   = displacement.z * Vector3::UNIT_Z;
        Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);

        Vector3 vertical_direction   = vertical_displacement.normalisedCopy();
        Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();

        Vector3 final_position = current_position;

        m_is_touch_ground = physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            Vector3::NEGATIVE_UNIT_Z,
            0.105f,
            hits);

        hits.clear();
        
        world_transform.m_position -= 0.1f * Vector3::UNIT_Z;

        // vertical pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            vertical_direction,
            vertical_displacement.length(),
            hits))
        {
            final_position += hits[0].hit_distance * vertical_direction;
        }
        else
        {
            final_position += vertical_displacement;
        }

        hits.clear();

        // side pass DONE
        float horizontal_length = horizontal_displacement.length();

        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix()/**** [0] ****/,
            horizontal_direction/**** [1] ****/,
            horizontal_length/**** [2] ****/,
            hits))
        {
            Vector3 horizontal_hit_normal = Vector3(hits[0].hit_normal.x, hits[0].hit_normal.y, 0.0f);
            float remaining_length = horizontal_length - hits[0].hit_distance;
            float horizontal_cos = horizontal_direction.dotProduct(horizontal_hit_normal);
            Vector3 horizontal_slide_vector = horizontal_direction - horizontal_hit_normal * horizontal_cos;

            final_position += hits[0].hit_distance * horizontal_direction/**** [3] ****/;

            Transform second_world_transform = world_transform;
            second_world_transform.m_position += Math::max(0.0f, hits[0].hit_distance - 0.25f) * horizontal_direction;
            std::vector<PhysicsHitInfo> second_hits;
            if (horizontal_slide_vector.length() > 0.0f && (!physics_scene->sweep(
                m_rigidbody_shape,
                second_world_transform.getMatrix()/**** [0] ****/,
                horizontal_slide_vector.normalisedCopy()/**** [1] ****/,
                remaining_length * horizontal_slide_vector.length()/**** [2] ****/,
                second_hits) || second_hits[0].body_id == hits[0].body_id))
            {
                final_position += horizontal_slide_vector * remaining_length;
            }
        }
        else
        {
            final_position += horizontal_displacement;
        }

        return final_position;
    }

} // namespace Pilot
