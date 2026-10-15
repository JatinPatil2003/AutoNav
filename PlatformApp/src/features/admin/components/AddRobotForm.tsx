import { useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../../components/ui/Button";
import { Input } from "../../../components/ui/Input";
import { getApiErrorMessage } from "../../../app/apiError";
import { useCreateRobotMutation } from "../adminApi";
import { AdminRobot } from "../admin.types";
import { CreateRobotFormValues, createRobotFormSchema } from "../admin.schemas";

/**
 * Minimal robot provisioning form for admins — pairs with the
 * PlatformServer/scripts/seedRobot.ts one-off script as the other way to
 * create a Robot record this milestone.
 */
export function AddRobotForm() {
  const [createRobot, { isLoading, error }] = useCreateRobotMutation();
  const [created, setCreated] = useState<AdminRobot | null>(null);
  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<CreateRobotFormValues>({ resolver: zodResolver(createRobotFormSchema) });

  async function onSubmit(values: CreateRobotFormValues) {
    try {
      const robot = await createRobot({
        robotId: values.robotId,
        name: values.name,
        vpnHostname: values.vpnHostname,
        displayName: values.displayName || undefined,
      }).unwrap();
      setCreated(robot);
      reset();
    } catch {
      // error state is surfaced below via the mutation's `error` field
    }
  }

  if (created) {
    return (
      <div className="flex flex-col gap-3">
        <div className="rounded-md bg-emerald-50 px-4 py-3 text-sm text-emerald-900 dark:bg-emerald-900/30 dark:text-emerald-200">
          <p className="mb-2 font-medium">Robot "{created.name}" created.</p>
          <p className="text-xs">
            No secret to copy — this robot identifies itself to PlatformServer with its own
            device id. Just make sure its ESP32 bridge's device id matches{" "}
            <code className="rounded bg-white px-1 py-0.5 dark:bg-slate-900">{created.robotId}</code>.
          </p>
        </div>
        <Button variant="secondary" onClick={() => setCreated(null)}>
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
