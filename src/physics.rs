use chaos_framework::Vec3;
use nalgebra::Vector;
use rapier3d::prelude::*;

pub struct RapierPhysicsWorld<'b> {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: &'b mut dyn BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub physics_hooks: (),
    pub event_handler: (),

    pub received_delta_time: Option<f32>,

    pub handles: Vec<RigidBodyHandle>,
}

impl<'b> RapierPhysicsWorld<'b> {
    pub fn new(broad_phase: &'b mut dyn BroadPhase) -> Self {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let mut handles = vec![];

        let gravity = vector![0.0, -9.8, 0.0];
        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let physics_hooks = ();
        let event_handler = ();

        Self {
            rigid_body_set,
            collider_set,
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            query_pipeline,
            physics_hooks,
            event_handler,
            handles,

            received_delta_time: Some(0.032),
        }
    }

    pub fn step(&mut self, g: Vec3) {
        self.integration_parameters.dt = self.received_delta_time.unwrap();

        self.physics_pipeline.step(
            &vector![g.x, g.y, g.z], // gravity
            &self.integration_parameters,
            &mut self.island_manager,
            self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.received_delta_time = Some(dt);
    }    

    pub fn add_cube_rigidbody(&mut self, pos: Vec3, size: Vec3) -> RigidBodyHandle {
        let size = size*0.5;
        let cube_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![pos.x, pos.y, pos.z])
            .build();
        let cube_collider = ColliderBuilder::cuboid(size.x, size.y, size.z).restitution(0.2).friction(3.0).build();
        let cube_body_handle = self.rigid_body_set.insert(cube_rigid_body.clone());

        self.handles.push(cube_body_handle.clone());
        self.collider_set.insert_with_parent(cube_collider.clone(), cube_body_handle, &mut self.rigid_body_set);

        return cube_body_handle;
    }

    pub fn add_static_cube_rigidbody(&mut self, pos: Vec3, size: Vec3) -> RigidBodyHandle {
        let size = size*0.5;
        let cube_rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![pos.x, pos.y, pos.z])
            .build();
        let cube_collider = ColliderBuilder::cuboid(size.x, size.y, size.z).restitution(0.2).friction(3.0).build();
        let cube_body_handle = self.rigid_body_set.insert(cube_rigid_body.clone());

        self.handles.push(cube_body_handle.clone());
        self.collider_set.insert_with_parent(cube_collider.clone(), cube_body_handle, &mut self.rigid_body_set);

        return cube_body_handle;
    }

    pub fn remove_rigidbody(&mut self, handle: RigidBodyHandle) {
        self.rigid_body_set.remove(
            handle, 
            &mut self.island_manager, 
            &mut self.collider_set, 
            &mut self.impulse_joint_set, 
            &mut self.multibody_joint_set, 
            true,
        );
    }
}