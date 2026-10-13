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
  const [createRobot, { isLoading, error, isSuccess }] = useCreateRobotMutation();
  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<CreateRobotFormValues>({ resolver: zodResolver(createRobotFormSchema) });

  async function onSubmit(values: CreateRobotFormValues) {
    try {
      await createRobot({
        name: values.name,
        vpnHostname: values.vpnHostname,
        displayName: values.displayName || undefined,
      }).unwrap();
      reset();
    } catch {
      // error state is surfaced below via the mutation's `error` field
    }
  }

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="flex flex-col gap-4">
      <Input
        label="Robot name"
        placeholder="autonav-02"
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
      {isSuccess && <p className="text-sm text-green-600 dark:text-green-400">Robot created.</p>}
      <Button type="submit" isLoading={isLoading} className="mt-1">
        Add robot
      </Button>
    </form>
  );
}
