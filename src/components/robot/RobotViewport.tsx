import { Canvas, useFrame } from "@react-three/fiber";
import { Grid, Line, OrbitControls, Text, Environment, Lightformer } from "@react-three/drei";
import { useMemo, useRef, useState } from "react";
import * as THREE from "three";
import {
  DEFAULT_ROBOT,
  forwardKinematics,
  type RobotConfig,
  type Vec3,
} from "@/lib/kinematics";

/** robot frame (Z up) -> three.js frame (Y up) */
function toThree(p: Vec3): [number, number, number] {
  return [p.x, p.z, -p.y];
}

const UP = new THREE.Vector3(0, 1, 0);

function Link({
  from,
  to,
  radius,
  color,
  emissive,
}: {
  from: Vec3;
  to: Vec3;
  radius: number;
  color: string;
  emissive: string;
}) {
  const a = new THREE.Vector3(...toThree(from));
  const b = new THREE.Vector3(...toThree(to));
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  const mid = new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5);
  const quat = new THREE.Quaternion().setFromUnitVectors(UP, dir.clone().normalize());
  if (len < 1e-6) return null;
  return (
    <group position={mid.toArray()} quaternion={quat.toArray() as unknown as THREE.Quaternion}>
      <mesh castShadow receiveShadow>
        <cylinderGeometry args={[radius, radius * 0.85, len, 24]} />
        <meshStandardMaterial
          color={color}
          emissive={emissive}
          emissiveIntensity={0.25}
          metalness={0.75}
          roughness={0.3}
        />
      </mesh>
    </group>
  );
}

function Axes({ size }: { size: number }) {
  const axes: { dir: [number, number, number]; color: string; label: string }[] = [
    { dir: [1, 0, 0], color: "#ff5c68", label: "X" },
    { dir: [0, 0, -1], color: "#4ade80", label: "Y" },
    { dir: [0, 1, 0], color: "#38bdf8", label: "Z" },
  ];
  return (
    <group>
      {axes.map((ax) => (
        <group key={ax.label}>
          <Line
            points={[
              [0, 0, 0],
              [ax.dir[0] * size, ax.dir[1] * size, ax.dir[2] * size],
            ]}
            color={ax.color}
            lineWidth={2}
          />
          <Text
            position={[
              ax.dir[0] * (size + 0.25),
              ax.dir[1] * (size + 0.25),
              ax.dir[2] * (size + 0.25),
            ]}
            fontSize={0.28}
            color={ax.color}
            anchorX="center"
            anchorY="middle"
          >
            {ax.label}
          </Text>
        </group>
      ))}
      {[1, 2, 3].map((t) => (
        <Text
          key={t}
          position={[t, 0.06, 0.22]}
          fontSize={0.14}
          color="#7b8aa3"
          anchorX="center"
        >
          {t.toString()}
        </Text>
      ))}
    </group>
  );
}

function TargetMarker({ target, reachable }: { target: Vec3; reachable: boolean }) {
  const ref = useRef<THREE.Group>(null);
  useFrame((state) => {
    if (ref.current) {
      const s = 1 + Math.sin(state.clock.elapsedTime * 3) * 0.08;
      ref.current.scale.setScalar(s);
      ref.current.rotation.y += 0.01;
    }
  });
  const color = reachable ? "#fbbf24" : "#f43f5e";
  return (
    <group position={toThree(target)}>
      <group ref={ref}>
        <mesh>
          <octahedronGeometry args={[0.16, 0]} />
          <meshStandardMaterial
            color={color}
            emissive={color}
            emissiveIntensity={1.2}
            transparent
            opacity={0.9}
          />
        </mesh>
        <mesh>
          <sphereGeometry args={[0.26, 20, 20]} />
          <meshBasicMaterial color={color} wireframe transparent opacity={0.35} />
        </mesh>
      </group>
    </group>
  );
}

function Robot({
  angles,
  config,
  trajectory,
  showTrajectory,
  onTip,
}: {
  angles: number[];
  config: RobotConfig;
  trajectory: Vec3[];
  showTrajectory: boolean;
  onTip: (p: Vec3) => void;
}) {
  const smoothed = useRef<number[]>([...angles]);
  const [, force] = useState(0);

  useFrame((_, delta) => {
    const k = 1 - Math.exp(-12 * Math.min(delta, 0.05));
    let moved = false;
    const next = smoothed.current.map((v, i) => {
      const t = angles[i] ?? 0;
      const nv = v + (t - v) * k;
      if (Math.abs(t - v) > 0.01) moved = true;
      return Math.abs(t - nv) < 0.005 ? t : nv;
    });
    smoothed.current = next;
    if (moved) {
      force((n) => n + 1);
      onTip(forwardKinematics(next, config).endEffector);
    }
  });

  const fk = forwardKinematics(smoothed.current, config);
  const j = fk.joints;
  const ee = fk.endEffector;

  const trajPoints = useMemo(
    () => trajectory.map((p) => toThree(p)),
    [trajectory],
  );

  return (
    <group>
      {/* base plinth */}
      <mesh position={[0, 0.06, 0]} receiveShadow castShadow>
        <cylinderGeometry args={[0.55, 0.68, 0.12, 40]} />
        <meshStandardMaterial color="#2a3342" metalness={0.8} roughness={0.35} />
      </mesh>
      <mesh position={[0, 0.16, 0]}>
        <torusGeometry args={[0.5, 0.02, 12, 48]} />
        <meshStandardMaterial color="#22d3ee" emissive="#22d3ee" emissiveIntensity={1.4} />
      </mesh>

      {j.slice(0, -1).map((p, i) => {
        const q = j[i + 1];
        if (!q) return null;
        const radii = [0.16, 0.13, 0.1];
        const colors = ["#93a4bd", "#8ea0ba", "#a8b6cc"];
        return (
          <Link
            key={i}
            from={p}
            to={q}
            radius={radii[i] ?? 0.1}
            color={colors[i] ?? "#93a4bd"}
            emissive="#0b1220"
          />
        );
      })}

      {j.slice(0, -1).map((p, i) => (
        <mesh key={`joint-${i}`} position={toThree(p)} castShadow>
          <sphereGeometry args={[0.19 - i * 0.03, 26, 26]} />
          <meshStandardMaterial
            color="#22d3ee"
            emissive="#0891b2"
            emissiveIntensity={0.6}
            metalness={0.6}
            roughness={0.25}
          />
        </mesh>
      ))}

      {/* end effector */}
      <group position={toThree(ee)}>
        <mesh castShadow>
          <sphereGeometry args={[0.13, 24, 24]} />
          <meshStandardMaterial color="#a78bfa" emissive="#7c3aed" emissiveIntensity={1.5} />
        </mesh>
        <mesh>
          <sphereGeometry args={[0.22, 16, 16]} />
          <meshBasicMaterial color="#a78bfa" wireframe transparent opacity={0.3} />
        </mesh>
      </group>

      {/* projection line to floor */}
      <Line
        points={[toThree(ee), [ee.x, 0, -ee.y]]}
        color="#a78bfa"
        lineWidth={1}
        dashed
        dashSize={0.08}
        gapSize={0.06}
        transparent
        opacity={0.5}
      />

      {showTrajectory && trajPoints.length > 1 && (
        <Line points={trajPoints} color="#f472b6" lineWidth={2} transparent opacity={0.85} />
      )}
    </group>
  );
}

export interface ViewportProps {
  angles: number[];
  config?: RobotConfig;
  target: Vec3;
  showTarget: boolean;
  reachable: boolean;
  trajectory: Vec3[];
  showTrajectory: boolean;
  onTip: (p: Vec3) => void;
}

export default function RobotViewport({
  angles,
  config = DEFAULT_ROBOT,
  target,
  showTarget,
  reachable,
  trajectory,
  showTrajectory,
  onTip,
}: ViewportProps) {
  const reach = config.L2 + config.L3;
  return (
    <Canvas shadows camera={{ position: [6.5, 5, 7], fov: 45 }} dpr={[1, 2]}>
      <color attach="background" args={["#080c14"]} />
      <fog attach="fog" args={["#080c14", 14, 30]} />
      <ambientLight intensity={0.45} />
      <directionalLight
        position={[6, 9, 5]}
        intensity={1.6}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <pointLight position={[-5, 3, -4]} intensity={30} color="#22d3ee" distance={18} />
      <Environment>
        <Lightformer intensity={1.6} position={[0, 6, 0]} scale={[10, 10, 1]} />
        <Lightformer
          intensity={0.9}
          color="#4c7bd1"
          position={[-6, 2, -2]}
          rotation-y={Math.PI / 2}
          scale={[16, 2, 1]}
        />
      </Environment>

      <Grid
        args={[20, 20]}
        cellSize={0.5}
        cellThickness={0.6}
        cellColor="#16304a"
        sectionSize={1}
        sectionThickness={1.1}
        sectionColor="#1d4e6b"
        fadeDistance={26}
        infiniteGrid
        position={[0, 0, 0]}
      />
      <mesh rotation-x={-Math.PI / 2} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[40, 40]} />
        <meshStandardMaterial color="#0a1018" metalness={0.2} roughness={0.9} />
      </mesh>

      {/* reach envelope */}
      <mesh position={[0, config.L1, 0]}>
        <sphereGeometry args={[reach, 32, 24]} />
        <meshBasicMaterial color="#1e88a8" wireframe transparent opacity={0.06} />
      </mesh>

      <Axes size={3} />
      <Robot
        angles={angles}
        config={config}
        trajectory={trajectory}
        showTrajectory={showTrajectory}
        onTip={onTip}
      />
      {showTarget && <TargetMarker target={target} reachable={reachable} />}

      <OrbitControls
        enableDamping
        dampingFactor={0.08}
        minDistance={2}
        maxDistance={20}
        target={[0, 1.2, 0]}
        maxPolarAngle={Math.PI / 2 - 0.02}
      />
    </Canvas>
  );
}
