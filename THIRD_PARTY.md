# Third-party models

lab_insert/models/fr3.xml is adapted from Google DeepMind's [MuJoCo Menagerie FR3 model](https://github.com/google-deepmind/mujoco_menagerie/tree/98a70c569c4c1b6da467a20012f766ee1f8dc521/franka_fr3). The eight link collision meshes are unmodified. The bundled XML omits cosmetic meshes and the home keyframe, and selects a 1 ms step. The included models/LICENSE reproduces the upstream license and notices. The procedural fingers are a project approximation.

[mjfit](https://github.com/younghyopark/mjfit) is installed as an external Python dependency at the revision in requirements.txt. Assembly geometry is generated at runtime.

