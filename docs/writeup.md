# Beaver's Bench: Learn to build a self-driving lab

Building a self-driving lab is complex, collaborative work, and AI on its own does not make learning it easier. Beaver's Bench pairs AI with a digital twin, a faithful 3D copy of a real robot bench, so people can learn to build and run an automated experiment platform before they touch the real one.

Beaver's Bench continues the colour-match training platform published in [Digital Discovery](https://doi.org/10.1039/d6dd00557h) (2026), a collaboration between the Hein lab, Telescope Innovations, who provide the physical platform, and IvoryOS, which orchestrates it. That platform teaches learners to turn a bench procedure into robot actions, but it cannot show the robot. People picture lab work as hand work: pick up the vial, uncap it, weigh it. A robot does each step as a chain of moves through safe and approach points, six of them just to place one vial. That gap between manual and automated work is where new builders stall, and closing it means seeing the movement.

## Approach

Beaver's Bench is a digital twin of the colour-match deck and its UR3e arm, written in TypeScript with three.js and served as an IvoryOS plugin. Every instrument call, such as `robotic_arm.pick_up_vial_from_tray(3)`, passes through the same pipeline:

1. **Input, from either of two sources.** In live mode, an IvoryOS workflow or Python script calls the lab's colour-match simulator, and a plugin bridge pushes each call and the deck state to the page over a websocket. In sandbox mode, a TypeScript port of the same simulator, with the same methods, checks and errors, runs in the browser with nothing to install.
2. **Choreography.** Each call expands into the moves the real arm controller makes: the home pose for that side of the deck, a joint move to the target's safe point, straight-line moves to its approach point and down to the target, the gripper, and back out. Station actions such as uncapping, solid and liquid dosing, and stirring by inverting the vial are scripted the same way.
3. **Deck as data.** Every pose comes from the deck's own location file. Parts the file does not cover are flagged "estimated" in the code and on screen.
4. **Animation.** Joint moves interpolate in joint space. Straight-line moves solve the UR3e's closed-form inverse kinematics every frame. Playback speeds up when the Python simulator runs ahead.
5. **Display.** The scene uses the UR3e model from ros-industrial. A panel beside it shows the call, its list of moves, the deck state and a log that reads like the control code.
6. **Verification.** The twin was built with Claude Code in one session, so it is checked rather than trusted. Tests compare the kinematics with the arm's reference model, round-trip every inverse kinematics solution, compare every pose with the location file, and drive the plugin through IvoryOS.

## Results

Beaver's Bench works today in both modes. The first version landed on 22 October 2026 after one day of work.

- Every command in the colour-match workflow is animated, including capping, powder dosing, liquid dosing, stirring and the camera check.
- It runs as a side panel inside IvoryOS, following each command the lab's simulator executes.
- The 28-step stock-solution demo plays end to end in the browser with no Python installed.
- 19 automated checks pass, covering the arm's geometry and every position against the lab's file. The twin's home pose matches the real arm within 1.5 cm.
- The hard part was not the graphics but fidelity. Where the file and the physical bench disagree, we had to decide which one the twin follows, and show that choice to the learner instead of hiding it.

## Next steps

AI can generate a lot of code quickly, but code written for one use and then abandoned is neither responsible nor sustainable. Beaver's Bench is built to be reused. It runs on the lab's published simulator and its real location file, plugs into IvoryOS where people already work, and is checked by tests so it stays true as the bench changes. Its purpose is to make sure the materials we already have, the training platform, the workbook and the deck, are actually used.

- **More instruments and modules.** Add instruments as the lab's decks grow, and extend the twin across the training platform's later modules.
- **Onboarding course and exercises.** Embed the browser version in the onboarding course, with the stock-solution demo as the first exercise and guided tasks after it.
- **Gamification.** Add challenges and progress tracking that turn practice into something learners come back to.
- **Shared with other labs.** Publish it on the IvoryOS Hub, and measure the parts still marked "estimated" so the twin keeps matching the real bench.
