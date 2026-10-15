import { PlaceholderPage } from "../../../components/ui/PlaceholderPage";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";

export function MappingPlaceholderPage() {
  const { selectedRobot } = useSelectedRobot();
  return (
    <PlaceholderPage
      title="Mapping"
      subtitle={selectedRobot ? `Robot: ${selectedRobot.displayName ?? selectedRobot.name}` : undefined}
    />
  );
}
