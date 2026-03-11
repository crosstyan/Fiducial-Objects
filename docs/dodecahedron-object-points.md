# Dodecahedron Marker Object Points

This note explains how the paper and this repository obtain marker object points for a dodecahedron-shaped fiducial object.

## Short Answer

The paper does not rely on UV unwrapping of a dodecahedron mesh, and it does not recover the as-built object by assuming an ideal regular dodecahedron.

Instead, it:

1. defines each marker only in its own local planar marker frame from the marker template,
2. estimates marker poses from many calibrated images,
3. converts co-visible marker poses into pairwise marker-to-marker transforms,
4. builds a pose graph over markers,
5. chooses one marker as the object reference frame, and
6. refines all marker poses by global reprojection-error minimization.

This means a good UV layout is not required for the paper's workflow.

## Local Marker Coordinates

Before any multiview reconstruction happens, each marker has a local coordinate system in its own face plane.

For custom polygon markers, the detector:

- reads the marker polygon from the marker configuration,
- recenters it,
- places it on `z = 0`, and
- scales it to the real marker size.

Relevant implementation:

- `src/utils/markerDetectors/cucomarkerMarkerDetector.hpp`
- `src/utils/generalMarkerDetector.cpp`

This local marker frame is enough to solve a pose for that marker in one image, provided the camera intrinsics are known.

## What Camera Intrinsics Give You

Camera intrinsics are required for the per-image pose solve.

For one image and one visible marker, the solver estimates the pose of that marker relative to the camera frame. In other words, the camera is the temporary reference frame for that single pose estimation.

However, there is no need to define a global object origin yet. The marker's local frame is enough for the single-image solve.

Relevant implementation:

- `3rdparty/markermapper/mappers/globalgraph_markermapper.cpp`

## Building the Co-Visibility Graph

The graph is not defined manually. It is built from detections across images.

The graph is:

- node: a marker ID
- edge `(i, j)`: markers `i` and `j` were seen together in at least one image
- edge data: one or more relative transforms between those two markers
- edge cost: reprojection-based quality of that relative transform

For each image:

1. detect all visible markers,
2. estimate each visible marker pose in the camera frame,
3. for every co-visible pair `(i, j)`, compute a relative transform between marker `i` and marker `j`,
4. store that transform as an observation on graph edge `(i, j)`.

After all images are processed, the implementation chooses the best representative relative transform for each marker pair by minimizing reprojection error across the available observations.

Relevant implementation:

- `3rdparty/markermapper/mappers/globalgraph_markermapper.cpp`
- `3rdparty/markermapper/mappers/globalgraph_markermapper.h`

## What "Connected" Means

The graph does not require one special reference marker to be visible in every image.

It only requires connectivity through overlap chains.

Example:

- frame 1 sees markers `{1, 2}`
- frame 2 sees markers `{2, 3, 4}`
- frame 3 sees markers `{4, 5}`

This is enough to place marker `5` relative to marker `1` through the chain `1 -> 2 -> 4 -> 5`, even though markers `1` and `5` never appear together.

The paper describes this as a pose quiver that is converted into a directed pose graph, followed by choosing one marker as the global reference and using a minimum spanning tree for the initial layout.

Relevant paper section:

- `docs/paper/sensors-23-09649.md`, Section 3.4

## ICP-Style Intuition

If you are thinking about Iterative Closest Point, that is a useful mental model.

At a systems level, the pattern is very similar:

- get local relative constraints from overlapping observations,
- assemble those constraints into a graph over rigid poses,
- pick an arbitrary reference frame to fix the global gauge freedom,
- optimize everything jointly for global consistency.

The difference is the measurement model.

Classic ICP usually uses:

- geometric overlap,
- nearest-neighbor correspondences,
- point-to-point or point-to-plane residuals.

This fiducial-object method uses:

- marker detections in images,
- known marker-corner identities within each marker,
- per-marker pose solves in the camera frame,
- pairwise marker-to-marker transforms from co-visible markers,
- final reprojection-error minimization over image points.

So this is not standard ICP, but it is conceptually similar to a registration pipeline followed by a global pose-graph or bundle-style refinement.

## "Pose Quiver" vs Pose Graph

The paper's term `pose quiver` is best read as the raw directed collection of pairwise relative-pose observations between markers gathered from all images.

After that, the implementation and the paper convert those observations into a cleaner pose graph:

- one node per marker,
- one consolidated relative transform per connected marker pair,
- one cost per edge expressing how well that transform explains the observations.

So the important idea is not the word `quiver`. The important idea is:

- first collect directed pairwise observations,
- then build and optimize a graph of marker poses.

## Correspondence Is the Real Prerequisite

For a bottom-up reconstruction idea such as "detect face geometry first, then infer the object," the key hidden prerequisite is correspondence.

You do not just need points. You need to know what those points belong to.

In practice, that usually means knowing at least one of these:

- which detected corners belong to the same face,
- which face a detected edge or corner belongs to,
- which observation in one frame matches which observation in another frame.

This is exactly where markers help. Marker IDs solve a large part of the correspondence problem up front:

- marker ID identifies the face observation,
- corner order identifies the local polygon corners,
- repeated observations of the same marker become easy to connect across images.

Without that, a rotating dodecahedron observed as unlabeled points, edges, or vertices is much harder to reconstruct robustly.

The reason is symmetry:

- many faces look the same,
- many edges and vertices are geometrically similar,
- even a correct rigid reconstruction can still suffer from face-label ambiguity.

If the object has strong local appearance cues, then generic multiview reconstruction can still work:

- detect repeatable local features,
- track or match them across views,
- estimate motion,
- triangulate and optimize,
- fit planar faces afterward.

But that is a different and harder pipeline. For symmetric polyhedra, the correspondence problem is often harder than the geometric optimization itself.

So a useful summary is:

- markers solve correspondence first,
- pose estimation and graph optimization solve geometry second.

## What Happens If a Face Is Never Visible

If a face is never seen in any calibration image, the paper's data-driven reconstruction cannot recover that face from observations alone.

Likewise, if a marker is seen but never connected to the rest of the marker set through any chain of co-visibility, it cannot be placed into the common object frame.

The implementation makes this practical requirement explicit by removing graph nodes with no connections.

Relevant implementation:

- `3rdparty/markermapper/mappers/globalgraph_markermapper.cpp`

This is why the paper's experiment setup matters: the authors captured images from many viewpoints and mounted the object in a way that allowed every face to be used fairly.

Relevant paper section:

- `docs/paper/sensors-23-09649.md`, Section 4.1

## What Frame Are the Final Object Points Relative To

The final object points are not stored relative to a camera frame.

They are stored relative to a common object frame, which is arbitrary up to a global rigid transform.

The paper and code fix this gauge freedom by choosing one marker as the reference marker. That marker's frame becomes the object frame, and all other marker corners are expressed relative to it.

So:

- temporary per-image reference frame: camera frame
- final persistent map frame: chosen object frame, usually one marker frame
- metric scale: comes from the supplied physical marker size

Relevant implementation:

- `3rdparty/markermapper/mappers/globalgraph_markermapper.cpp`
- `utils_developer/mapperObject/mapperObject.cpp`

The repository also includes an example calibrated dodecahedron map:

- `DATA/COMPLETE/Dodecahedron/Map/output.yml`

## How This Differs From UV Mapping

The UV-based workflow in `ChArUcoBoardExp` is a different method.

That workflow assumes:

- a trusted mesh,
- a trusted UV atlas,
- correct image-to-UV alignment, and
- a known metric scale for the mesh.

It then transfers marker corners from UV space back to 3D mesh space, which is a CAD/mesh-driven approach.

The paper's workflow here is different:

- it does not need a UV map,
- it does not trust the ideal CAD placement as the final truth,
- it reconstructs the actual assembled object from multiview observations.

This distinction matters because printed and glued markers rarely end up in the exact ideal positions.

## How Standard Is the UV-First Idea

The exact workflow "UV map -> marker corners -> object points for fiducial markers" is not what this paper proposes.

However, the geometric operation underneath that idea is well established in graphics and mesh-processing systems:

- represent surface points by triangle-local barycentric coordinates,
- interpolate attributes such as UV coordinates from mesh vertices,
- or invert that relationship by using parametric or UV coordinates to recover positions or attributes on the surface.

This is standard enough that mainstream geometry/rendering tools expose it directly:

- Houdini documents primitive UV/UVW coordinates as a way of referring to positions on surfaces and interpolating geometry attributes, and its `primuv` function returns interpolated attribute values at given primitive coordinates.
- NVIDIA's `nvdiffrast` documentation describes rasterization and interpolation in barycentric coordinates, with texture coordinates and other per-vertex attributes interpolated across triangles.

There is also academic language treating barycentric UV/mesh transfer as a common mesh-processing operation. For example, one texture-mapping paper states that barycentric mapping is commonly used for mapping 3D meshes onto the UV domain.

Reference examples:

- https://www.sidefx.com/docs/houdini/model/primitive_spaces.html
- https://nvlabs.github.io/nvdiffrast/
- https://www.mdpi.com/2076-3417/15/11/6288

So the safest wording is:

- not necessarily "standard" as a named fiducial-marker pipeline,
- but standard as a low-level graphics / geometry operation,
- and therefore a very natural engineering idea if you already trust the mesh and UV atlas.

## What Is Actually Novel Here

The graph-based reconstruction pattern itself is not the main novelty. Building relative pose constraints from overlapping observations and then optimizing them globally is a standard idea in vision, registration, and SLAM-style systems.

The paper's stated contributions are elsewhere:

- using custom polygon markers that fit the polyhedron faces instead of forcing square markers onto them,
- extending custom marker design into a full 3D fiducial-object workflow,
- estimating the actual marker configuration of the assembled object from multiview images,
- evaluating several fiducial-object shapes under noise, blur, scale, and occlusion.

Relevant paper section:

- `docs/paper/sensors-23-09649.md`, Introduction contributions paragraph

So the novelty is better understood as:

- not "inventing pose graphs",
- but applying a graph-based multiview calibration workflow to custom polygonal fiducial objects whose real assembled geometry is not perfectly known in advance.

## Practical Guidance

If your goal is the real as-built dodecahedron, use the paper's multiview calibration approach.

If your goal is an ideal regular dodecahedron with mathematically exact faces, use an analytic polyhedron model.

If you already have a correct mesh and UVs, UV transfer is a valid alternative, but it is a different workflow and solves a different problem.
