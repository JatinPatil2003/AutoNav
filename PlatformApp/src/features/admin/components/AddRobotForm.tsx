import { useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../../components/ui/Button";
import { Input } from "../../../components/ui/Input";
import { getApiErrorMessage } from "../../../app/apiError";
import { useCreateRobotMutation } from "../adminApi";
import { CreateRobotFormValues, createRobotFormSchema } from "../admin.schemas";

/**
 * Minimal robot provisioning form for admins — pairs with the
 * PlatformServer/scripts/seedRobot.ts one-off script as the other way to
 * create a Robot record this milestone.
 */
export function AddRobotForm() {
  const [createRobot, { isLoading, error }] = useCreateRobotMutation();
  const [revealedSecret, setRevealedSecret] = useState<string | null>(null);
  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<CreateRobotFormValues>({ resolver: zodResolver(createRobotFormSchema) });

  async function onSubmit(values: CreateRobotFormValues) {
    try {
      const result = await createRobot({
        robotId: values.robotId,
        name: values.name,
        vpnHostname: values.vpnHostname,
        displayName: values.displayName || undefined,
      }).unwrap();
      setRevealedSecret(result.robotSecret);
      reset();
    } catch {
      // error state is surfaced below via the mutation's `error` field
    }
  }

  if (revealedSecret) {
    return (
      <div className="flex flex-col gap-3">
        <div className="rounded-md bg-amber-50 px-4 py-3 text-sm text-amber-900 dark:bg-amber-900/30 dark:text-amber-200">
          <p className="mb-2 font-medium">Copy this robot secret now — it won't be shown again.</p>
          <code className="block select-all break-all rounded bg-white px-2 py-1.5 text-xs dark:bg-slate-900">
            {revealedSecret}
          </code>
          <p className="mt-2 text-xs">
            Paste it into this robot's <code>RobotServer/.env</code> as{" "}
            <code>PLATFORM_ROBOT_SECRET</code>, alongside the matching <code>PLATFORM_ROBOT_ID</code>.
          </p>
        </div>
        <Button variant="secondary" onClick={() => setRevealedSecret(null)}>
          Add another robot
        </Button>
      </div>
    );
  }

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="flex flex-col gap-4">
      <Input
        label="Robot ID"
        placeholder="autonav-02"
        error={errors.robotId?.message}
        {...register("robotId")}
      />
      <Input
        label="Robot name"
        placeholder="Autonav 02"
        error={errors.name?.message}
        {...register("name")}
      />
      <Input
        label="VPN hostname"
        placeholder="autonav-02.robot"
        error={errors.vpnHostname?.message}
        {...register("vpnHostname")}
      />
      <Input
        label="Display name (optional)"
        placeholder="Warehouse Robot 2"
        error={errors.displayName?.message}
        {...register("displayName")}
      />
      {error && <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>}
      <Button type="submit" isLoading={isLoading} className="mt-1">
        Add robot
      </Button>
    </form>
  );
}
