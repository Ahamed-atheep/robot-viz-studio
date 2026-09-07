import { createFileRoute } from "@tanstack/react-router";
import { lazy, Suspense, useCallback, useMemo, useRef, useState } from "react";
import {
  DEFAULT_ROBOT,
  distance,
  fmt,
  forwardKinematics,
  inverseKinematics,
  clamp,
  type Vec3,
} from "@/lib/kinematics";

const RobotViewport = lazy(() => import("@/components/robot/RobotViewport"));

export const Route = createFileRoute("/")({
  ssr: false,
  head: () => ({
    meta: [
      { title: "3D Kinematics Dashboard — FK & IK Robotic Arm Simulator" },
      {
        name: "description",
        content:
          "Interactive 3D robotics dashboard for forward and inverse kinematics: live joint control, DH transformation matrices, target solving and end-effector trajectory.",
      },
      { property: "og:title", content: "3D Kinematics Dashboard — FK & IK Robotic Arm" },
      {
        property: "og:description",
        content:
          "Simulate a 3-DOF robotic arm in real time with forward and inverse kinematics, transformation matrices and trajectory tracking.",
      },
      { property: "og:type", content: "website" },
      { name: "twitter:card", content: "summary_large_image" },
    ],
  }),
  component: Dashboard,
});

const CFG = DEFAULT_ROBOT;
const DEFAULT_ANGLES = [30, 45, -60];
const MAX_TRAJ = 260;

type Mode = "FK" | "IK";

function Dashboard() {
  const [mode, setMode] = useState<Mode>("FK");
  const [angles, setAngles] = useState<number[]>(DEFAULT_ANGLES);
  const [target, setTarget] = useState<Vec3>({ x: 1.6, y: 0.9, z: 1.8 });
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [trajectory, setTrajectory] = useState<Vec3[]>([]);
  const [ikInfo, setIkInfo] = useState<{ reachable: boolean; reason?: string } | null>(null);
  const lastPoint = useRef<Vec3 | null>(null);

  const fk = useMemo(() => forwardKinematics(angles, CFG), [angles]);
  const ik = useMemo(() => inverseKinematics(target, CFG), [target]);
  const eePos = fk.endEffector;
  const posError = mode === "IK" ? distance(eePos, target) : 0;

  const handleTip = useCallback((p: Vec3) => {
    const prev = lastPoint.current;
    if (!prev || distance(prev, p) > 0.02) {
      lastPoint.current = p;
      setTrajectory((t) => {
        const next = [...t, p];
        return next.length > MAX_TRAJ ? next.slice(next.length - MAX_TRAJ) : next;
      });
    }
  }, []);

  const setAngle = (i: number, v: number) =>
    setAngles((a) => a.map((x, idx) => (idx === i ? v : x)));

  const solveIK = () => {
    const sol = inverseKinematics(target, CFG);
    setIkInfo({ reachable: sol.reachable && sol.withinLimits, reason: sol.reason });
    if (sol.reachable) setAngles(sol.angles);
    setMode("IK");
  };

  const reset = () => {
    setAngles(DEFAULT_ANGLES);
    setTarget({ x: 1.6, y: 0.9, z: 1.8 });
    setTrajectory([]);
    setIkInfo(null);
    setMode("FK");
  };

  const reachable = ik.reachable && ik.withinLimits;

  return (
    <div className="min-h-screen bg-background text-foreground">
      <header className="flex flex-wrap items-center justify-between gap-3 border-b border-border bg-panel/70 px-5 py-3 backdrop-blur">
        <div className="flex items-center gap-3">
          <div className="grid h-9 w-9 place-items-center rounded-md bg-accent/15 text-accent ring-1 ring-accent/40">
            <span className="font-mono text-sm font-bold">RX</span>
          </div>
          <div>
            <h1 className="font-mono text-sm font-semibold tracking-[0.18em] text-foreground uppercase">
              Kinematics Control Dashboard
            </h1>
            <p className="text-xs text-muted-foreground">
              {CFG.name} · 3-DOF RRR · DH parameterised
            </p>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <ModeSwitch mode={mode} onChange={setMode} />
          <button
            onClick={() => setTrajectory([])}
            className="rounded-md border border-border px-3 py-1.5 font-mono text-xs text-muted-foreground transition-colors hover:border-accent/60 hover:text-accent"
          >
            CLEAR PATH
          </button>
          <button
            onClick={reset}
            className="rounded-md bg-accent/15 px-3 py-1.5 font-mono text-xs font-semibold text-accent ring-1 ring-accent/40 transition-colors hover:bg-accent/25"
          >
            RESET
          </button>
        </div>
      </header>

      <main className="grid gap-4 p-4 xl:grid-cols-[minmax(0,1fr)_380px]">
        <section className="flex flex-col gap-4">
          <div className="relative h-[52vh] min-h-[380px] overflow-hidden rounded-xl border border-border bg-viewport shadow-panel xl:h-[calc(100vh-190px)]">
            <Suspense
              fallback={
                <div className="grid h-full place-items-center font-mono text-xs text-muted-foreground">
                  INITIALISING 3D WORKSPACE…
                </div>
              }
            >
              <RobotViewport
                angles={angles}
                target={target}
                showTarget={mode === "IK"}
                reachable={reachable}
                trajectory={trajectory}
                showTrajectory={showTrajectory}
                onTip={handleTip}
              />
            </Suspense>

            <div className="pointer-events-none absolute left-4 top-4 space-y-1 font-mono text-[11px] text-muted-foreground">
              <div className="text-accent">● END EFFECTOR</div>
              <div>
                X {fmt(eePos.x)} &nbsp; Y {fmt(eePos.y)} &nbsp; Z {fmt(eePos.z)}
              </div>
            </div>
            <div className="pointer-events-none absolute bottom-4 left-4 flex flex-wrap gap-3 font-mono text-[10px] text-muted-foreground">
              <Legend color="#93a4bd" label="LINKS" />
              <Legend color="#22d3ee" label="JOINTS" />
              <Legend color="#a78bfa" label="END EFFECTOR" />
              <Legend color="#fbbf24" label="TARGET" />
              <Legend color="#f472b6" label="TRAJECTORY" />
            </div>
            <div className="pointer-events-none absolute bottom-4 right-4 font-mono text-[10px] text-muted-foreground">
              DRAG · ROTATE &nbsp;|&nbsp; SCROLL · ZOOM &nbsp;|&nbsp; RIGHT-DRAG · PAN
            </div>
          </div>

          <StatusBar
            mode={mode}
            ee={eePos}
            target={target}
            error={posError}
            angles={angles}
            reachable={reachable}
            showTrajectory={showTrajectory}
            onToggleTrajectory={() => setShowTrajectory((s) => !s)}
          />
        </section>

        <aside className="flex flex-col gap-4">
          <Panel title="Forward Kinematics" accent={mode === "FK"}>
            <div className="space-y-4">
              {angles.map((a, i) => (
                <Slider
                  key={i}
                  label={`θ${i + 1}`}
                  value={a}
                  min={CFG.limits[i]?.min ?? -180}
                  max={CFG.limits[i]?.max ?? 180}
                  onChange={(v) => {
                    setMode("FK");
                    setAngle(i, v);
                  }}
                />
              ))}
              <div className="grid grid-cols-3 gap-2 pt-1">
                <Readout label="X" value={fmt(eePos.x)} />
                <Readout label="Y" value={fmt(eePos.y)} />
                <Readout label="Z" value={fmt(eePos.z)} />
              </div>
              <div>
                <div className="mb-1 font-mono text-[10px] tracking-widest text-muted-foreground">
                  HOMOGENEOUS TRANSFORM T⁰₃
                </div>
                <div className="rounded-md border border-border bg-surface p-2 font-mono text-[10px] leading-relaxed">
                  {fk.transform.map((row, r) => (
                    <div key={r} className="grid grid-cols-4 gap-1">
                      {row.map((v, c) => (
                        <span
                          key={c}
                          className={
                            c === 3 ? "text-right text-accent" : "text-right text-foreground/80"
                          }
                        >
                          {fmt(v, 2)}
                        </span>
                      ))}
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </Panel>

          <Panel title="Inverse Kinematics" accent={mode === "IK"}>
            <div className="space-y-3">
              <div className="grid grid-cols-3 gap-2">
                {(["x", "y", "z"] as const).map((k) => (
                  <label key={k} className="block">
                    <span className="font-mono text-[10px] tracking-widest text-muted-foreground">
                      TARGET {k.toUpperCase()}
                    </span>
                    <input
                      type="number"
                      step={0.1}
                      value={target[k]}
                      onChange={(e) =>
                        setTarget((t) => ({
                          ...t,
                          [k]: clamp(Number(e.target.value) || 0, -10, 10),
                        }))
                      }
                      className="mt-1 w-full rounded-md border border-border bg-surface px-2 py-1.5 font-mono text-sm text-foreground outline-none focus:border-accent"
                    />
                  </label>
                ))}
              </div>

              <button
                onClick={solveIK}
                className="w-full rounded-md bg-accent px-3 py-2 font-mono text-xs font-bold tracking-widest text-accent-foreground transition-opacity hover:opacity-90"
              >
                SOLVE &amp; MOVE
              </button>

              <div
                className={`rounded-md border px-3 py-2 font-mono text-[11px] ${
                  reachable
                    ? "border-ok/40 bg-ok/10 text-ok"
                    : "border-danger/40 bg-danger/10 text-danger"
                }`}
              >
                {reachable
                  ? "✓ TARGET REACHABLE"
                  : `✕ UNREACHABLE — ${ikInfo?.reason ?? ik.reason ?? "no valid solution"}`}
              </div>

              <div className="grid grid-cols-3 gap-2">
                <Readout label="θ1" value={`${fmt(ik.angles[0] ?? 0, 2)}°`} />
                <Readout label="θ2" value={`${fmt(ik.angles[1] ?? 0, 2)}°`} />
                <Readout label="θ3" value={`${fmt(ik.angles[2] ?? 0, 2)}°`} />
              </div>
              <div className="grid grid-cols-2 gap-2">
                <Readout
                  label="TARGET"
                  value={`${fmt(target.x, 2)}, ${fmt(target.y, 2)}, ${fmt(target.z, 2)}`}
                />
                <Readout
                  label="ACHIEVED"
                  value={`${fmt(ik.achieved.x, 2)}, ${fmt(ik.achieved.y, 2)}, ${fmt(ik.achieved.z, 2)}`}
                />
              </div>
            </div>
          </Panel>

          <Panel title="Robot Configuration">
            <div className="grid grid-cols-2 gap-2 font-mono text-[11px]">
              <Readout label="BASE L1" value={`${CFG.L1} m`} />
              <Readout label="LINK L2" value={`${CFG.L2} m`} />
              <Readout label="LINK L3" value={`${CFG.L3} m`} />
              <Readout label="MAX REACH" value={`${fmt(CFG.L2 + CFG.L3, 2)} m`} />
            </div>
          </Panel>
        </aside>
      </main>
    </div>
  );
}

function Legend({ color, label }: { color: string; label: string }) {
  return (
    <span className="flex items-center gap-1.5">
      <span className="h-2 w-2 rounded-full" style={{ backgroundColor: color }} />
      {label}
    </span>
  );
}

function ModeSwitch({ mode, onChange }: { mode: Mode; onChange: (m: Mode) => void }) {
  return (
    <div className="flex rounded-md border border-border p-0.5">
      {(["FK", "IK"] as Mode[]).map((m) => (
        <button
          key={m}
          onClick={() => onChange(m)}
          className={`rounded px-3 py-1 font-mono text-xs tracking-widest transition-colors ${
            mode === m
              ? "bg-accent text-accent-foreground"
              : "text-muted-foreground hover:text-foreground"
          }`}
        >
          {m}
        </button>
      ))}
    </div>
  );
}

function Panel({
  title,
  children,
  accent,
}: {
  title: string;
  children: React.ReactNode;
  accent?: boolean;
}) {
  return (
    <div
      className={`rounded-xl border bg-panel p-4 shadow-panel transition-colors ${
        accent ? "border-accent/50" : "border-border"
      }`}
    >
      <h2 className="mb-3 font-mono text-[11px] font-semibold tracking-[0.2em] text-muted-foreground uppercase">
        {title}
      </h2>
      {children}
    </div>
  );
}

function Slider({
  label,
  value,
  min,
  max,
  onChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  onChange: (v: number) => void;
}) {
  return (
    <div>
      <div className="flex items-baseline justify-between font-mono text-xs">
        <span className="text-foreground">{label}</span>
        <span className="text-accent">{fmt(value, 1)}°</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={0.5}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="range-track mt-1.5 w-full"
      />
      <div className="flex justify-between font-mono text-[10px] text-muted-foreground">
        <span>{min}°</span>
        <span>{max}°</span>
      </div>
    </div>
  );
}

function Readout({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-md border border-border bg-surface px-2 py-1.5">
      <div className="font-mono text-[9px] tracking-widest text-muted-foreground">{label}</div>
      <div className="font-mono text-xs text-foreground">{value}</div>
    </div>
  );
}

function StatusBar({
  mode,
  ee,
  target,
  error,
  angles,
  reachable,
  showTrajectory,
  onToggleTrajectory,
}: {
  mode: Mode;
  ee: Vec3;
  target: Vec3;
  error: number;
  angles: number[];
  reachable: boolean;
  showTrajectory: boolean;
  onToggleTrajectory: () => void;
}) {
  return (
    <div className="flex flex-wrap items-center gap-3 rounded-xl border border-border bg-panel p-3 shadow-panel">
      <Chip label="MODE" value={mode === "FK" ? "FORWARD KINEMATICS" : "INVERSE KINEMATICS"} />
      <Chip label="EE" value={`${fmt(ee.x, 2)}, ${fmt(ee.y, 2)}, ${fmt(ee.z, 2)}`} />
      <Chip label="TARGET" value={`${fmt(target.x, 2)}, ${fmt(target.y, 2)}, ${fmt(target.z, 2)}`} />
      <Chip label="ERROR" value={`${fmt(error, 4)} m`} />
      <Chip
        label="θ"
        value={angles.map((a) => `${fmt(a, 1)}°`).join("  ")}
      />
      <span
        className={`rounded-md px-2 py-1 font-mono text-[10px] tracking-widest ${
          reachable ? "bg-ok/15 text-ok" : "bg-danger/15 text-danger"
        }`}
      >
        {reachable ? "REACHABLE" : "OUT OF RANGE"}
      </span>
      <button
        onClick={onToggleTrajectory}
        className={`ml-auto rounded-md border px-3 py-1.5 font-mono text-[10px] tracking-widest transition-colors ${
          showTrajectory
            ? "border-trail/50 bg-trail/10 text-trail"
            : "border-border text-muted-foreground"
        }`}
      >
        TRAJECTORY {showTrajectory ? "ON" : "OFF"}
      </button>
    </div>
  );
}

function Chip({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-md border border-border bg-surface px-2.5 py-1">
      <span className="font-mono text-[9px] tracking-widest text-muted-foreground">{label}</span>
      <div className="font-mono text-[11px] text-foreground">{value}</div>
    </div>
  );
}
